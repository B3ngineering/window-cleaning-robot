#!/usr/bin/env python3
"""
Navigation Node for Window Cleaning Robot

Implements boustrophedon (snake) path cleaning:
1. Move UP a vertical path in CLEANING mode
2. Move DOWN the same path in CHECKING mode (camera verification)
3. If clean: shift horizontally, repeat
4. If dirty: retry cleaning (up to max_attempts)
"""
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from window_cleaner.msg import RobotState, CleaningResult
from window_cleaner.srv import SetCleaningArea, SetCleaningAreaResponse
from window_cleaner.srv import AbortMission, AbortMissionResponse
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional


class NavigationMode(Enum):
    IDLE = 0
    CLEANING = 1      # Moving upward, cleaning head active
    CHECKING = 2      # Moving downward, camera checking
    PAUSED = 3        # E-stop active, waiting to resume


@dataclass
class Waypoint:
    """A single point in the cleaning path."""
    x: float
    y: float
    column: int       # Which vertical column (stripe) this belongs to
    is_top: bool      # True if this is the top of a column


@dataclass 
class CleaningStripe:
    """A vertical stripe to clean."""
    column: int
    x: float
    y_bottom: float
    y_top: float
    attempts: int = 0
    is_clean: bool = False


class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node')

        # Load configuration parameters
        self.margin = rospy.get_param('/margin', 0.1)
        self.robot_width = rospy.get_param('/robot_width', 0.2)
        self.robot_height = rospy.get_param('/robot_height', 0.3)
        self.confidence_threshold = rospy.get_param('/confidence_threshold', 0.5)
        self.max_clean_attempts = rospy.get_param('/max_clean_attempts', 3)
        self.home_x = rospy.get_param('/home_position_x', 0.78)
        self.home_y = rospy.get_param('/home_position_y', 0.66)
        self.default_spacing = rospy.get_param('/default_clean_spacing', 0.15)

        # Navigation state
        self.mode = NavigationMode.IDLE
        self.robot_status = RobotState.STATUS_IDLE
        self.previous_status = RobotState.STATUS_IDLE
        
        # Path planning state
        self.stripes: List[CleaningStripe] = []
        self.current_stripe_idx = 0
        self.current_waypoint: Optional[Waypoint] = None
        self.waiting_for_arrival = False
        self.waiting_for_result = False
        
        # Cleaning area bounds
        self.area_x_min = 0.0
        self.area_x_max = 0.0
        self.area_y_min = 0.0
        self.area_y_max = 0.0
        
        # Statistics
        self.total_stripes = 0
        self.completed_stripes = 0
        self.failed_stripes = []
        
        # Robot position tracking
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Publishers
        self.target_pub = rospy.Publisher('/target_position', Point, queue_size=10)
        self.capture_pub = rospy.Publisher('/capture_request', Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('/robot_state', RobotState, self.robot_state_cb)
        rospy.Subscriber('/cleaning_result', CleaningResult, self.cleaning_result_cb)

        # Services
        rospy.Service('set_cleaning_area', SetCleaningArea, self.handle_set_cleaning_area)
        rospy.Service('abort_mission', AbortMission, self.handle_abort_mission)

        # Position logging timer (1 Hz)
        self.position_log_rate = rospy.get_param('/position_log_rate', 1.0)
        rospy.Timer(rospy.Duration(1.0 / self.position_log_rate), self.log_position)

        rospy.loginfo("Navigation Node initialized")
        rospy.loginfo(f"  Margin: {self.margin}m, Robot: {self.robot_width}x{self.robot_height}m")
        rospy.loginfo(f"  Max attempts per stripe: {self.max_clean_attempts}")

    # =========================================================================
    # Service Handlers
    # =========================================================================
    
    def handle_set_cleaning_area(self, req):
        """
        Generate boustrophedon cleaning path from requested area.
        
        Pattern: vertical stripes, each stripe width = robot_width / 2 for overlap
        """
        rospy.loginfo(f"SetCleaningArea: {req.width}m x {req.height}m, spacing={req.pass_spacing}m")
        
        # Validate request
        if req.width <= 0 or req.height <= 0:
            return SetCleaningAreaResponse(
                accepted=False, 
                num_passes=0,
                message="Invalid dimensions"
            )
        
        # Use provided spacing or default
        pass_spacing = req.pass_spacing if req.pass_spacing > 0 else self.default_spacing
        
        # Calculate cleaning area bounds (with margin)
        # Assume cleaning area is centered in frame for now
        frame_center_x = rospy.get_param('/pulley_separation_x', 1.56) / 2
        frame_center_y = rospy.get_param('/pulley_separation_y', 1.32) / 2
        
        self.area_x_min = frame_center_x - req.width / 2 + self.margin
        self.area_x_max = frame_center_x + req.width / 2 - self.margin
        self.area_y_min = frame_center_y - req.height / 2 + self.margin
        self.area_y_max = frame_center_y + req.height / 2 - self.margin
        
        # Ensure bounds respect robot size
        self.area_x_min = max(self.area_x_min, self.robot_width / 2)
        self.area_y_min = max(self.area_y_min, self.robot_height / 2)
        
        rospy.loginfo(f"  Cleaning bounds: X[{self.area_x_min:.2f}, {self.area_x_max:.2f}] "
                     f"Y[{self.area_y_min:.2f}, {self.area_y_max:.2f}]")
        
        # Generate vertical stripes (boustrophedon pattern)
        # Stripe width = half robot width for 50% overlap
        stripe_width = self.robot_width / 2
        self.stripes = []
        
        x = self.area_x_min
        column = 0
        while x <= self.area_x_max:
            stripe = CleaningStripe(
                column=column,
                x=x,
                y_bottom=self.area_y_min,
                y_top=self.area_y_max,
                attempts=0,
                is_clean=False
            )
            self.stripes.append(stripe)
            x += stripe_width
            column += 1
        
        self.total_stripes = len(self.stripes)
        self.current_stripe_idx = 0
        self.completed_stripes = 0
        self.failed_stripes = []
        
        rospy.loginfo(f"  Generated {self.total_stripes} vertical stripes")
        
        # Start cleaning sequence
        if self.total_stripes > 0:
            self.start_cleaning_stripe()
        
        return SetCleaningAreaResponse(
            accepted=True,
            num_passes=self.total_stripes,
            message=f"Starting cleaning with {self.total_stripes} stripes"
        )

    def handle_abort_mission(self, req):
        """Abort current cleaning mission."""
        rospy.logwarn("AbortMission requested")
        
        self.mode = NavigationMode.IDLE
        self.stripes = []
        self.current_stripe_idx = 0
        self.waiting_for_arrival = False
        self.waiting_for_result = False
        
        message = "Mission aborted"
        
        # Optionally return to home position
        if req.return_home:
            rospy.loginfo(f"Returning to home position: ({self.home_x}, {self.home_y})")
            self.send_target(self.home_x, self.home_y)
            message += ", returning home"
        
        return AbortMissionResponse(accepted=True, message=message)

    # =========================================================================
    # Cleaning State Machine
    # =========================================================================
    
    def start_cleaning_stripe(self):
        """Begin cleaning the current stripe by moving to bottom, then UP."""
        if self.current_stripe_idx >= len(self.stripes):
            self.finish_mission()
            return
        
        stripe = self.stripes[self.current_stripe_idx]
        stripe.attempts += 1
        
        rospy.loginfo(f"Starting stripe {stripe.column + 1}/{self.total_stripes} "
                     f"(attempt {stripe.attempts}/{self.max_clean_attempts})")
        
        # Move to bottom of stripe first
        self.mode = NavigationMode.CLEANING
        self.current_waypoint = Waypoint(
            x=stripe.x, 
            y=stripe.y_bottom,
            column=stripe.column,
            is_top=False
        )
        self.waiting_for_arrival = True
        self.send_target(stripe.x, stripe.y_bottom)
    
    def move_up_stripe(self):
        """Move upward along current stripe in CLEANING mode."""
        stripe = self.stripes[self.current_stripe_idx]
        
        rospy.loginfo(f"CLEANING: Moving UP stripe {stripe.column + 1}")
        self.mode = NavigationMode.CLEANING
        self.current_waypoint = Waypoint(
            x=stripe.x,
            y=stripe.y_top,
            column=stripe.column,
            is_top=True
        )
        self.waiting_for_arrival = True
        # TODO: Activate cleaning head here
        self.send_target(stripe.x, stripe.y_top)
    
    def move_down_stripe_checking(self):
        """Move downward along current stripe in CHECKING mode."""
        stripe = self.stripes[self.current_stripe_idx]
        
        rospy.loginfo(f"CHECKING: Moving DOWN stripe {stripe.column + 1}")
        self.mode = NavigationMode.CHECKING
        self.current_waypoint = Waypoint(
            x=stripe.x,
            y=stripe.y_bottom,
            column=stripe.column,
            is_top=False
        )
        self.waiting_for_arrival = True
        self.send_target(stripe.x, stripe.y_bottom)
    
    def advance_to_next_stripe(self):
        """Move to the next stripe or finish mission."""
        self.current_stripe_idx += 1
        self.completed_stripes += 1
        
        if self.current_stripe_idx >= len(self.stripes):
            self.finish_mission()
        else:
            rospy.loginfo(f"Advancing to stripe {self.current_stripe_idx + 1}/{self.total_stripes}")
            self.start_cleaning_stripe()
    
    def retry_current_stripe(self):
        """Retry cleaning the current stripe."""
        stripe = self.stripes[self.current_stripe_idx]
        
        if stripe.attempts >= self.max_clean_attempts:
            rospy.logwarn(f"Stripe {stripe.column + 1} failed after {stripe.attempts} attempts, moving on")
            self.failed_stripes.append(stripe.column)
            self.advance_to_next_stripe()
        else:
            rospy.loginfo(f"Retrying stripe {stripe.column + 1} "
                         f"(attempt {stripe.attempts + 1}/{self.max_clean_attempts})")
            self.start_cleaning_stripe()
    
    def finish_mission(self):
        """Complete the cleaning mission."""
        self.mode = NavigationMode.IDLE
        
        if self.failed_stripes:
            rospy.logwarn(f"Mission complete with {len(self.failed_stripes)} failed stripes: {self.failed_stripes}")
        else:
            rospy.loginfo(f"Mission complete! Cleaned {self.completed_stripes} stripes successfully")
        
        # Return home
        rospy.loginfo(f"Returning to home position")
        self.send_target(self.home_x, self.home_y)

    # =========================================================================
    # Subscriber Callbacks
    # =========================================================================
    
    def robot_state_cb(self, msg):
        """Handle robot state updates from motion controller."""
        self.previous_status = self.robot_status
        self.robot_status = msg.status
        
        # Update robot position
        self.robot_x = msg.x
        self.robot_y = msg.y
        
        # Handle E-STOP
        if msg.status == RobotState.STATUS_STOP:
            if self.mode != NavigationMode.PAUSED and self.mode != NavigationMode.IDLE:
                rospy.logwarn("E-STOP detected, pausing navigation")
                self.mode = NavigationMode.PAUSED
            return
        
        # Resume from E-STOP
        if self.mode == NavigationMode.PAUSED and msg.status != RobotState.STATUS_STOP:
            rospy.loginfo("E-STOP cleared, resuming from current stripe")
            self.mode = NavigationMode.CLEANING
            self.start_cleaning_stripe()
            return
        
        # Handle arrival at target
        if msg.status == RobotState.STATUS_AT_TARGET and self.waiting_for_arrival:
            self.waiting_for_arrival = False
            self.on_arrival_at_waypoint()
    
    def on_arrival_at_waypoint(self):
        """Called when robot arrives at current waypoint."""
        if self.current_waypoint is None:
            return
        
        rospy.loginfo(f"Arrived at waypoint: ({self.current_waypoint.x:.2f}, {self.current_waypoint.y:.2f})")
        
        if self.mode == NavigationMode.CLEANING:
            if self.current_waypoint.is_top:
                # Reached top of stripe in cleaning mode
                # Now switch to checking mode and go back down
                rospy.loginfo("Reached top, starting camera check on way down")
                self.move_down_stripe_checking()
            else:
                # Reached bottom, now move up while cleaning
                self.move_up_stripe()
                
        elif self.mode == NavigationMode.CHECKING:
            if not self.current_waypoint.is_top:
                # Reached bottom after checking - request camera verification
                rospy.loginfo("Reached bottom, requesting cleanliness check")
                self.request_cleanliness_check()
    
    def request_cleanliness_check(self):
        """Request vision node to check if stripe is clean."""
        self.waiting_for_result = True
        capture_msg = Bool()
        capture_msg.data = True
        self.capture_pub.publish(capture_msg)
        rospy.loginfo("Published capture_request")
    
    def cleaning_result_cb(self, msg):
        """Handle cleaning result from vision node."""
        if not self.waiting_for_result:
            return
        
        self.waiting_for_result = False
        stripe = self.stripes[self.current_stripe_idx]
        
        rospy.loginfo(f"Cleaning result: success={msg.clean_successful}, "
                     f"confidence={msg.confidence:.2f}")
        
        # Check if clean with sufficient confidence
        if msg.clean_successful and msg.confidence >= self.confidence_threshold:
            rospy.loginfo(f"Stripe {stripe.column + 1} verified CLEAN")
            stripe.is_clean = True
            self.advance_to_next_stripe()
        else:
            reason = msg.failure_reason if msg.failure_reason else "insufficient confidence"
            rospy.logwarn(f"Stripe {stripe.column + 1} NOT clean: {reason}")
            self.retry_current_stripe()

    # =========================================================================
    # Helper Methods
    # =========================================================================
    
    def log_position(self, event):
        """Periodically log the robot's center position."""
        status_names = {
            RobotState.STATUS_IDLE: "IDLE",
            RobotState.STATUS_MOVING: "MOVING",
            RobotState.STATUS_AT_TARGET: "AT_TARGET",
            RobotState.STATUS_STOP: "STOP",
            RobotState.STATUS_ERROR: "ERROR"
        }
        status_str = status_names.get(self.robot_status, "UNKNOWN")
        rospy.loginfo(f"Robot position: ({self.robot_x:.3f}, {self.robot_y:.3f}) [{status_str}]")
    
    def send_target(self, x: float, y: float):
        """Publish target position to motion controller."""
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        self.target_pub.publish(point)
        rospy.loginfo(f"Published target: ({x:.3f}, {y:.3f})")


if __name__ == '__main__':
    try:
        NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass