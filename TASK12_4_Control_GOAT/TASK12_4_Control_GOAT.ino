// PID Constants
float Kp = 1.0, Ki = 0.5, Kd = 0.1;
float setpoint = 100.0; // Desired speed of the motor
float input = 0.0, output = 0.0;
float previous_error = 0.0, integral = 0.0;
float smoothedOutput = 0.0;
float alpha = 0.1; // Smoothing factor for Exponential Smoothing Filter
unsigned long lastTime = 0;

void setup() {
  pinMode(9, OUTPUT); // PWM pin to control motor speed
  Serial.begin(9600); // Initialize serial communication for debugging
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds

  // Read current speed (example input from analog pin A0)
  input = analogRead(A0);

  // PID
  float error = setpoint - input;
  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;

  // exponential filter
  smoothedOutput = alpha * output + (1 - alpha) * smoothedOutput;

  // Constrain the smoothed output to the valid PWM range
  smoothedOutput = constrain(smoothedOutput, 0, 255);

  // Apply smoothed output to motor
  analogWrite(9, smoothedOutput);

  // Debug output to monitor values
  Serial.print("Setpoint: "); Serial.print(setpoint);
  Serial.print("input: "); Serial.print(input);
  Serial.print("PID output: "); Serial.print(output);
  Serial.print("smoothed output: "); Serial.println(smoothedOutput);

  // Update previous error and time
  previous_error = error;
  lastTime = currentTime;

  // Add a small delay to avoid overwhelming the serial output
  delay(100);
}
