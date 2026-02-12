#!/usr/bin/env python3
"""
Simple interactive demo of the cable robot simulator
"""

import numpy as np
import matplotlib.pyplot as plt
from cable_robot_simulator import (
    RobotConfig, CableRobotSimulator, 
    plot_robot_state, plot_simulation_results
)


def demo_basic_movement():
    """Demo 1: Basic movement to a single point"""
    print("\n" + "="*60)
    print("DEMO 1: Basic Movement")
    print("="*60)
    
    config = RobotConfig()
    sim = CableRobotSimulator(config)
    
    print("\nStarting position:", sim.ee_position)
    print("Cable lengths:", sim.cable_lengths)
    
    # Move to top-right
    target = np.array([1.5, 1.5])
    print(f"\nMoving to {target}...")
    
    sim.run_to_target(target, timeout=5.0)
    
    print("\nFinal position:", sim.ee_position)
    print("Final cable lengths:", sim.cable_lengths)
    
    plot_robot_state(sim)
    plt.savefig('demo1_basic_movement.png', dpi=150, bbox_inches='tight')
    print("\nSaved: demo1_basic_movement.png")


def demo_square_path():
    """Demo 2: Square path trajectory"""
    print("\n" + "="*60)
    print("DEMO 2: Square Path Trajectory")
    print("="*60)
    
    config = RobotConfig()
    sim = CableRobotSimulator(config)
    
    # Define square corners
    square = [
        np.array([0.6, 0.6]),  # Bottom-left
        np.array([0.6, 1.4]),  # Top-left
        np.array([1.4, 1.4]),  # Top-right
        np.array([1.4, 0.6]),  # Bottom-right
        np.array([0.6, 0.6]),  # Back to start
    ]
    
    print("\nTracing a square path...")
    for i, point in enumerate(square):
        print(f"  Corner {i+1}: {point}")
        sim.run_to_target(point, timeout=3.0)
    
    plot_simulation_results(sim)
    plt.savefig('demo2_square_path.png', dpi=150, bbox_inches='tight')
    print("\nSaved: demo2_square_path.png")


def demo_manual_control():
    """Demo 3: Manual step-by-step control"""
    print("\n" + "="*60)
    print("DEMO 3: Manual Step-by-Step Control")
    print("="*60)
    
    config = RobotConfig()
    sim = CableRobotSimulator(config)
    
    # Set target
    target = np.array([1.5, 0.5])
    sim.set_desired_position(target)
    
    print(f"\nTarget: {target}")
    print("Running 500 simulation steps manually...\n")
    
    # Run fixed number of steps
    for i in range(500):
        sim.step()
        
        # Print progress every 100 steps
        if i % 100 == 0:
            error = np.linalg.norm(sim.ee_position - sim.desired_position)
            print(f"Step {i}: Position = {sim.ee_position}, Error = {error:.4f} m")
    
    plot_robot_state(sim)
    plt.savefig('demo3_manual_control.png', dpi=150, bbox_inches='tight')
    print("\nSaved: demo3_manual_control.png")


def demo_kinematics_test():
    """Demo 4: Test forward and inverse kinematics"""
    print("\n" + "="*60)
    print("DEMO 4: Kinematics Test")
    print("="*60)
    
    config = RobotConfig()
    sim = CableRobotSimulator(config)
    
    # Test position
    test_pos = np.array([1.2, 0.8])
    
    print(f"\nTest position: {test_pos}")
    
    # Inverse kinematics
    cable_lengths = sim.kinematics.inverse_kinematics(test_pos)
    print(f"Cable lengths (IK): {cable_lengths}")
    
    # Forward kinematics
    recovered_pos = sim.kinematics.forward_kinematics_iterative(cable_lengths)
    print(f"Recovered position (FK): {recovered_pos}")
    
    # Check error
    error = np.linalg.norm(test_pos - recovered_pos)
    print(f"FK/IK Round-trip error: {error:.6f} m")
    
    if error < 1e-4:
        print("✓ Kinematics test PASSED")
    else:
        print("✗ Kinematics test FAILED")


def demo_workspace_limits():
    """Demo 5: Test workspace boundaries"""
    print("\n" + "="*60)
    print("DEMO 5: Workspace Boundary Test")
    print("="*60)
    
    config = RobotConfig()
    sim = CableRobotSimulator(config)
    
    # Test various positions
    test_positions = [
        (np.array([1.0, 1.0]), "Center"),
        (np.array([0.2, 0.2]), "Near corner (valid)"),
        (np.array([0.05, 0.05]), "Too close to corner (invalid)"),
        (np.array([1.0, 2.1]), "Above window (invalid)"),
        (np.array([1.0, 0.5]), "Lower middle (valid)"),
    ]
    
    print("\nTesting positions:")
    for pos, description in test_positions:
        valid = sim.kinematics.check_workspace_limits(pos)
        status = "✓ VALID" if valid else "✗ INVALID"
        print(f"  {description:30s} {pos} : {status}")


def demo_controller_tuning():
    """Demo 6: Compare different controller gains"""
    print("\n" + "="*60)
    print("DEMO 6: Controller Tuning Comparison")
    print("="*60)
    
    target = np.array([1.5, 0.7])
    
    # Test different PID gains
    gain_sets = [
        {"kp": 1.0, "ki": 0.05, "kd": 0.3, "name": "Slow"},
        {"kp": 2.0, "ki": 0.1, "kd": 0.5, "name": "Default"},
        {"kp": 4.0, "ki": 0.2, "kd": 1.0, "name": "Fast"},
    ]
    
    fig, axes = plt.subplots(len(gain_sets), 1, figsize=(10, 12))
    
    for i, gains in enumerate(gain_sets):
        print(f"\nTesting {gains['name']} controller: Kp={gains['kp']}, Ki={gains['ki']}, Kd={gains['kd']}")
        
        config = RobotConfig()
        sim = CableRobotSimulator(config)
        
        # Set custom PID gains
        for controller in sim.controllers:
            controller.kp = gains['kp']
            controller.ki = gains['ki']
            controller.kd = gains['kd']
        
        # Run simulation
        sim.run_to_target(target, timeout=5.0)
        
        # Plot position error
        time = np.array(sim.history['time'])
        ee_pos = np.array(sim.history['ee_pos'])
        desired_pos = np.array(sim.history['desired_pos'])
        error = np.linalg.norm(ee_pos - desired_pos, axis=1)
        
        axes[i].plot(time, error, linewidth=2)
        axes[i].set_ylabel('Error (m)')
        axes[i].set_title(f'{gains["name"]} Controller (Kp={gains["kp"]}, Ki={gains["ki"]}, Kd={gains["kd"]})')
        axes[i].grid(True, alpha=0.3)
        
        if i == len(gain_sets) - 1:
            axes[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig('demo6_controller_tuning.png', dpi=150, bbox_inches='tight')
    print("\nSaved: demo6_controller_tuning.png")


def run_all_demos():
    """Run all demonstrations"""
    print("\n" + "="*70)
    print(" "*15 + "CABLE ROBOT SIMULATOR - ALL DEMOS")
    print("="*70)
    
    demo_basic_movement()
    demo_square_path()
    demo_manual_control()
    demo_kinematics_test()
    demo_workspace_limits()
    demo_controller_tuning()
    
    print("\n" + "="*70)
    print("All demos complete! Check the generated PNG files.")
    print("="*70)
    
    plt.show()


if __name__ == "__main__":
    # You can run individual demos or all at once
    
    # Option 1: Run all demos
    run_all_demos()
    
    # Option 2: Run individual demos (comment out run_all_demos() above)
    # demo_basic_movement()
    # demo_kinematics_test()
    # etc.