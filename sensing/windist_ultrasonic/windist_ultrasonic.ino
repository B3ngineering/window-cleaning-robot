const int trigPin = 9;
const int echoPin = 10;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Clear trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 10 µs pulse to start measurement
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the echo pulse length
  long duration = pulseIn(echoPin, HIGH, 30000UL); // timeout 30ms (≈5 meters)

  if (duration == 0) {
    Serial.println("Out of range");
  } else {
    // Convert time to distance
    float distance_cm = duration / 58.0;
    Serial.print("Distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
  }

  delay(200);
}
