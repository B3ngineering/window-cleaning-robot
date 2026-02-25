#define NUM_SENSORS 4

// Assign pins
int trigPins[NUM_SENSORS] = {2, 4, 6, 8};
int echoPins[NUM_SENSORS] = {3, 5, 7, 9};

float distances[NUM_SENSORS];

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

// float readSensor(int trigPin, int echoPin) {

//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);

//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(20);

//   digitalWrite(trigPin, LOW);

//   // 25ms timeout (~4m max range)
//   long duration = pulseIn(echoPin, HIGH, 25000);

//   if (duration == 0) {
//     return -1;  // No echo received
//   }

//   float distance = (duration / 2.0) * 0.343; // in mm
//   return distance;
// }

float readSensor(int trigPin, int echoPin) {
  // MEDIAN VERSION

  float readings[3];
  int validCount = 0;

  for (int i = 0; i < 3; i++) {

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20);

    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 25000);

    if (duration > 0) {
      readings[validCount] = (duration / 2.0) * 0.343;
      validCount++;
    }

    delay(10);
  }

  // If no valid readings
  if (validCount == 0) {
    return -1;
  }

  // If only one valid reading
  if (validCount == 1) {
    return readings[0];
  }

  // If two valid readings → return smaller (safer for obstacle avoidance)
  if (validCount == 2) {
    return min(readings[0], readings[1]);
  }

  // If three valid readings → sort and return middle (median)

  // Simple bubble sort for 3 elements
  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (readings[j] < readings[i]) {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  return readings[1];  // middle value
}

void loop() {

  for (int i = 0; i < NUM_SENSORS; i++) {

    distances[i] = readSensor(trigPins[i], echoPins[i]);

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");

    if (distances[i] < 0) {
      Serial.println("No echo");
    } else {
      Serial.print(distances[i]);
      Serial.println(" mm");
    }

    delay(60);  // Prevent cross-talk
  }

  Serial.println("----");
}