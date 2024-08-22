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

  /*
// Set PWM frequency for Timer1 (pins 9 and 10)
  setPWMFrequency256(10); // Set to 122.55 Hz
  setPWMFrequency256(11); // Set to 122.55 Hz

  // Set PWM frequency for Timer0 (pins 5 and 6)
  setPWMFrequency256(5); // Set to 122.55 Hz
  setPWMFrequency256(6); // Set to 122.55 Hz
  */

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
      //digitalWrite(MOTOR_LEFT_EN_FORWARD, HIGH);
      //digitalWrite(MOTOR_LEFT_EN_BACKWARD, LOW);
      digitalWrite(MOTOR_LEFT_EN_FORWARD, HIGH);
      digitalWrite(MOTOR_LEFT_EN_BACKWARD, HIGH);
      analogWrite(MOTOR_LEFT_PWM_BACKWARD, 0);
      delay(10);
      analogWrite(MOTOR_LEFT_PWM_FORWARD, speed);
      break;
    case 'B':  // Backward
      //digitalWrite(MOTOR_LEFT_EN_FORWARD, LOW);
      //digitalWrite(MOTOR_LEFT_EN_BACKWARD, HIGH);
      digitalWrite(MOTOR_LEFT_EN_FORWARD, HIGH);
      digitalWrite(MOTOR_LEFT_EN_BACKWARD, HIGH);
      analogWrite(MOTOR_LEFT_PWM_FORWARD, 0);
      delay(10);
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
      digitalWrite(MOTOR_RIGHT_EN_BACKWARD, HIGH);
      //digitalWrite(MOTOR_RIGHT_EN_BACKWARD, LOW);
      analogWrite(MOTOR_RIGHT_PWM_BACKWARD, 0);
      delay(10);
      analogWrite(MOTOR_RIGHT_PWM_FORWARD, speed);

      break;
    case 'B':  // Backward
      //digitalWrite(MOTOR_RIGHT_EN_FORWARD, LOW);
      //digitalWrite(MOTOR_RIGHT_EN_BACKWARD, HIGH);
      digitalWrite(MOTOR_RIGHT_EN_FORWARD, HIGH);
      digitalWrite(MOTOR_RIGHT_EN_BACKWARD, HIGH);
      analogWrite(MOTOR_RIGHT_PWM_FORWARD, 0);
      delay(10);
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

// Function to set PWM frequency for a specific pin
void setPWMFrequency(int pin, int frequency) {
  int prescaler;
  if (frequency < 31)       prescaler = 0x05;  // Prescale = 1024
  else if (frequency < 62)  prescaler = 0x04;  // Prescale = 256
  else if (frequency < 250) prescaler = 0x03;  // Prescale = 64
  else if (frequency < 1000) prescaler = 0x02; // Prescale = 8
  else                     prescaler = 0x01;  // Prescale = 1 (no prescaling)

  // Set Timer1 to the desired frequency
  TCCR1B = TCCR1B & 0b11111000 | prescaler;

  int16_t topValue = (16000000 / (frequency * 2 * (1 << (prescaler - 1)))) - 1;
  ICR1 = topValue; // Set the TOP value for Timer1
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (TCCR1B & 0b11111000 | prescaler);

  // Set Timer0 for PWM pins 5 and 6
  if (pin == 5 || pin == 6) {
    TCCR0B = (TCCR0B & 0b11111000) | prescaler;
  }
}

// Function to set PWM frequency for Timer1 and Timer0
void setPWMFrequency256(int pin) {
  // Set Timer1 for pins 9 and 10
  if (pin == 10 || pin == 11) {
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);  // Mode 14 (fast PWM), Prescaler 256
    ICR1 = 504;  // Set TOP value for 122.55 Hz
  }

  // Set Timer0 for pins 5 and 6
  if (pin == 5 || pin == 6) {
    TCCR0B = (TCCR0B & 0b11111000) | 0x04;  // Prescaler 256
    OCR0A = 124;  // Set TOP value for 122.55 Hz (for pins 5 and 6)
  }
}
