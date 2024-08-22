#include <Wire.h>

#define MOTOR_LEFT_PWM_FORWARD 11
#define MOTOR_LEFT_PWM_BACKWARD 10
#define MOTOR_LEFT_EN_FORWARD 9
#define MOTOR_LEFT_EN_BACKWARD 8

#define MOTOR_RIGHT_PWM_FORWARD 6
#define MOTOR_RIGHT_PWM_BACKWARD 5
#define MOTOR_RIGHT_EN_FORWARD 4
#define MOTOR_RIGHT_EN_BACKWARD 3

#define I2C_ADDRESS 0x08  // I2C address for Arduino Pro Mini
#define MAX_INPUT_LENGTH 6  // Max length of input string (e.g., "LF255" = 5 characters + null terminator)

char inputString[MAX_INPUT_LENGTH];  // Buffer for incoming string
int inputIndex = 0;  // Current position in the buffer
bool inputComplete = false;  // Whether the input string is complete

void setup() {
  // Set motor control pins as outputs
  pinMode(MOTOR_LEFT_PWM_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_PWM_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_EN_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_EN_BACKWARD, OUTPUT);

  pinMode(MOTOR_RIGHT_PWM_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_EN_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_EN_BACKWARD, OUTPUT);

  // Initialize I2C communication
  Wire.begin(I2C_ADDRESS);  // Join I2C bus with address
  Wire.onReceive(receiveEvent);  // Register event for data reception

  // Initialize Serial communication
  Serial.begin(9600);

  stopMotors();
  Serial.println("Setup complete. Motors stopped.");
}

void loop() {
  // Check if a complete string has been received via Serial
  if (inputComplete) {
    processInputString();
    inputComplete = false;
  }

  // Check for incoming serial data
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      inputString[inputIndex] = '\0';  // Terminate the string
      inputComplete = true;
      inputIndex = 0;  // Reset index for the next command
    } else {
      if (inputIndex < MAX_INPUT_LENGTH - 1) {
        inputString[inputIndex++] = inChar;
      }
    }
  }
}

// Function to handle I2C data reception
void receiveEvent(int howMany) {
  if (howMany >= 3 && howMany < MAX_INPUT_LENGTH) {
    for (int i = 0; i < howMany; i++) {
      inputString[i] = Wire.read();
    }
    inputString[howMany] = '\0';  // Null-terminate the string
    processInputString();
  }
}

// Process the input string
void processInputString() {
  if (strlen(inputString) < 3) {
    Serial.println("Invalid input");
    return;  // Not a valid command
  }

  char motor = inputString[0];  // Motor identifier ('L' or 'R')
  char command = inputString[1];  // Command for motor ('F', 'B', 'S')
  int speed = atoi(&inputString[2]);  // Convert the rest of the string to an integer for speed

  // Validate motor and command
  if ((motor == 'L' || motor == 'R') && (command == 'F' || command == 'B' || command == 'S')) {
    processCommand(motor, command, speed);
  } else {
    Serial.println("Invalid command");
  }
}

// Process the command and send appropriate debug message
void processCommand(char motor, char command, int speed) {
  if (speed > 255) speed = 255;  // Cap speed at 255
  if (speed < 0) speed = 0;      // Ensure speed is non-negative

  if (motor == 'L') {
    controlLeftMotor(command, speed);
    Serial.print("Left motor ");
  } else if (motor == 'R') {
    controlRightMotor(command, speed);
    Serial.print("Right motor ");
  }

  switch (command) {
    case 'F':
      Serial.print("forward ");
      break;
    case 'B':
      Serial.print("backward ");
      break;
    case 'S':
      Serial.print("stopped ");
      break;
  }

  Serial.print("with speed ");
  Serial.println(speed);
}

void controlLeftMotor(char command, int speed) {
  switch (command) {
    case 'F':  // Forward
      digitalWrite(MOTOR_LEFT_EN_FORWARD, HIGH);
      digitalWrite(MOTOR_LEFT_EN_BACKWARD, LOW);
      analogWrite(MOTOR_LEFT_PWM_FORWARD, speed);
      analogWrite(MOTOR_LEFT_PWM_BACKWARD, 0);
      break;
    case 'B':  // Backward
      digitalWrite(MOTOR_LEFT_EN_FORWARD, LOW);
      digitalWrite(MOTOR_LEFT_EN_BACKWARD, HIGH);
      analogWrite(MOTOR_LEFT_PWM_FORWARD, 0);
      analogWrite(MOTOR_LEFT_PWM_BACKWARD, speed);
      break;
    case 'S':  // Stop
      digitalWrite(MOTOR_LEFT_EN_FORWARD, LOW);
      digitalWrite(MOTOR_LEFT_EN_BACKWARD, LOW);
      analogWrite(MOTOR_LEFT_PWM_FORWARD, 0);
      analogWrite(MOTOR_LEFT_PWM_BACKWARD, 0);
      break;
  }
}

void controlRightMotor(char command, int speed) {
  switch (command) {
    case 'F':  // Forward
      digitalWrite(MOTOR_RIGHT_EN_FORWARD, HIGH);
      digitalWrite(MOTOR_RIGHT_EN_BACKWARD, LOW);
      analogWrite(MOTOR_RIGHT_PWM_FORWARD, speed);
      analogWrite(MOTOR_RIGHT_PWM_BACKWARD, 0);
      break;
    case 'B':  // Backward
      digitalWrite(MOTOR_RIGHT_EN_FORWARD, LOW);
      digitalWrite(MOTOR_RIGHT_EN_BACKWARD, HIGH);
      analogWrite(MOTOR_RIGHT_PWM_FORWARD, 0);
      analogWrite(MOTOR_RIGHT_PWM_BACKWARD, speed);
      break;
    case 'S':  // Stop
      digitalWrite(MOTOR_RIGHT_EN_FORWARD, LOW);
      digitalWrite(MOTOR_RIGHT_EN_BACKWARD, LOW);
      analogWrite(MOTOR_RIGHT_PWM_FORWARD, 0);
      analogWrite(MOTOR_RIGHT_PWM_BACKWARD, 0);
      break;
  }
}

void stopMotors() {
  // Stop both motors
  controlLeftMotor('S', 0);
  controlRightMotor('S', 0);
  Serial.println("All motors stopped.");
}
