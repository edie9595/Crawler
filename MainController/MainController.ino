#include <Wire.h>

#define I2C_ADDRESS 0x08  // I2C address for Arduino Pro Mini

// Define pins for RC channels
#define RC_CHANNEL_1_PIN 34  // Throttle (e.g., forward/backward)
#define RC_CHANNEL_2_PIN 35  // Steering (e.g., left/right)

// Timing variables
volatile uint32_t rc_channel_1_value = 1500;  // Neutral position
volatile uint32_t rc_channel_2_value = 1500;  // Neutral position

void setup() {
  // Initialize I2C
  Wire.begin();

  // Setup the pins as input
  pinMode(RC_CHANNEL_1_PIN, INPUT);
  pinMode(RC_CHANNEL_2_PIN, INPUT);

  // Attach interrupt to capture PWM signals
  attachInterrupt(digitalPinToInterrupt(RC_CHANNEL_1_PIN), readChannel1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CHANNEL_2_PIN), readChannel2, CHANGE);

  Serial.begin(115200);
  Serial.println("ESP32 RC Mixer Started");
}

void loop() {
  // Mix the RC signals to create tank drive commands
  int left_motor_speed, right_motor_speed;
  mixChannels(rc_channel_1_value, rc_channel_2_value, left_motor_speed, right_motor_speed);

  // Send the commands via I2C
  sendI2CCommand('L', left_motor_speed);
  sendI2CCommand('R', right_motor_speed);

  delay(50);  // Send commands at a regular interval
}

// Function to capture RC Channel 1 (Throttle)
void readChannel1() {
  static uint32_t last_interrupt_time = 0;
  uint32_t current_time = micros();
  if (digitalRead(RC_CHANNEL_1_PIN) == HIGH) {
    last_interrupt_time = current_time;
  } else {
    rc_channel_1_value = current_time - last_interrupt_time;
  }
}

// Function to capture RC Channel 2 (Steering)
void readChannel2() {
  static uint32_t last_interrupt_time = 0;
  uint32_t current_time = micros();
  if (digitalRead(RC_CHANNEL_2_PIN) == HIGH) {
    last_interrupt_time = current_time;
  } else {
    rc_channel_2_value = current_time - last_interrupt_time;
  }
}

// Function to mix throttle and steering for tank control
void mixChannels(int throttle, int steering, int &left_motor, int &right_motor) {
  int throttle_offset = map(throttle, 1000, 2000, -255, 255);  // Map throttle to -255 to 255
  int steering_offset = map(steering, 1000, 2000, -255, 255);  // Map steering to -255 to 255

  left_motor = constrain(throttle_offset + steering_offset, -255, 255);
  right_motor = constrain(throttle_offset - steering_offset, -255, 255);

  // Convert to unsigned speed (0-255) and set direction
  left_motor = (left_motor >= 0) ? left_motor : (256 + left_motor);
  right_motor = (right_motor >= 0) ? right_motor : (256 + right_motor);
}

// Function to send motor command via I2C
void sendI2CCommand(char motor, int speed) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(motor);  // 'L' for left motor, 'R' for right motor
  Wire.write((speed >= 256) ? 'B' : 'F');  // 'F' for forward, 'B' for backward
  Wire.write(abs(speed % 256));  // Speed value
  Wire.endTransmission();
}
