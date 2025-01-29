#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"

// Pin definitions
#define CURRENT_SENSOR_PIN 5
#define CAN_RX_PIN 2
#define CAN_TX_PIN 1
#define MOTOR_PWM_PIN 3
#define MOTOR_INA_PIN 20
#define MOTOR_INB_PIN 8
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_REGISTER 0x0E // High byte of the angle register
#define SDA_PIN 9
#define SCL_PIN 10

// TWAI Configuration
twai_timing_config_t twai_timing_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t twai_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
twai_general_config_t twai_general_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);

// LEDC channel and timer definitions
#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_0 0
#define LEDC_BASE_FREQ 14000 // 14 kHz

// Define Axis States
#define AXIS_STATE_IDLE 0x00
#define AXIS_STATE_CLOSED_LOOP 0x08
#define AXIS_STATE_MOTOR_CALIBRATION 0x01

// Global variable to store the current axis state
uint32_t currentAxisState = AXIS_STATE_IDLE;

// Global PID gain
float kp = 1.0;

// Message Handling Functions
void SetAxisState(uint8_t* data);
void SetInputPosition(uint8_t* data);
void SetInputVelocity(uint8_t* data);
void SetInputTorque(uint8_t* data);
void SetLimits(uint8_t* data);
void SetControllerModes(uint8_t* data);
void SetTrajectoryParameters(uint8_t* data);
void RequestEncoderEstimate(uint8_t* data);
void RequestErrorState(uint8_t* data);
void ClearErrors(uint8_t* data);

// Helper functions
void initializeTWAI();
void processCANMessage(twai_message_t& message);
float readAS5600();
void controlMotor(float pidOutput);

void setup() {
  Serial.begin(115200);
  delay(1000); // Delay to allow time for serial monitor to connect

  // Initialize I2C for AS5600
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize TWAI (CAN)
  initializeTWAI();

  // Configure LEDC PWM for motor control
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, 8); // 8-bit resolution
  ledcAttachPin(MOTOR_PWM_PIN, LEDC_CHANNEL_0); // Attach the PWM pin to the channel

  // Configure motor control pins as outputs
  pinMode(MOTOR_INA_PIN, OUTPUT);
  pinMode(MOTOR_INB_PIN, OUTPUT);

  Serial.println("ESP32-C3 Motor Controller Initialized.");
}

void loop() {
  twai_message_t message;

  // Check for received CAN messages
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    processCANMessage(message);
  }
}

// Initialize TWAI
void initializeTWAI() {
  if (twai_driver_install(&twai_general_config, &twai_timing_config, &twai_filter_config) == ESP_OK) {
    Serial.println("TWAI driver installed.");
  } else {
    Serial.println("Failed to install TWAI driver.");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("TWAI driver started.");
  } else {
    Serial.println("Failed to start TWAI driver.");
  }
}

// Process incoming CAN message
void processCANMessage(twai_message_t& message) {
  switch (message.identifier) {
    case 0x007: // Set Axis State
      SetAxisState(message.data);
      Serial.printf("Received Set Axis State: 0x%08X\n", *((uint32_t*)message.data));
      break;
    case 0x00C: // Set Input Position
      SetInputPosition(message.data);
      Serial.printf("Received Set Input Position: %f\n", *((float*)message.data));
      break;
    case 0x00D: // Set Input Velocity
      SetInputVelocity(message.data);
      Serial.printf("Received Set Input Velocity: %f\n", *((float*)message.data));
      break;
    case 0x00E: // Set Input Torque
      SetInputTorque(message.data);
      Serial.printf("Received Set Input Torque: %f\n", *((float*)message.data));
      break;
    case 0x00F: // Set Limits
      SetLimits(message.data);
      Serial.printf("Received Set Limits: %f, %f\n", *((float*)message.data), *((float*)(message.data + 4)));
      break;
    case 0x010: // Set Controller Modes
      SetControllerModes(message.data);
      Serial.printf("Received Set Controller Modes: 0x%08X, 0x%08X\n", *((uint32_t*)message.data), *((uint32_t*)(message.data + 4)));
      break;
    case 0x011: // Set Trajectory Parameters
      SetTrajectoryParameters(message.data);
      Serial.printf("Received Set Trajectory Parameters: %f, %f, %f\n", *((float*)message.data), *((float*)(message.data + 4)), *((float*)(message.data + 8)));
      break;
    case 0x009: // Request Encoder Estimate
      RequestEncoderEstimate(message.data);
      Serial.println("Received Request Encoder Estimate.");
      break;
    case 0x008: // Request Error State
      RequestErrorState(message.data);
      Serial.println("Received Request Error State.");
      break;
    case 0x017: // Clear Errors
      ClearErrors(message.data);
      Serial.println("Received Clear Errors.");
      break;
    default:
      Serial.printf("Unknown message ID: 0x%03X\n", message.identifier);
      break;
  }
}

// Handling Functions
void SetAxisState(uint8_t* data) {
  uint32_t axis_state = *((uint32_t*)data);
  Serial.printf("Handling Set Axis State: 0x%08X\n", axis_state);

  // Validate the axis_state value
  if (axis_state != AXIS_STATE_IDLE && axis_state != AXIS_STATE_CLOSED_LOOP && axis_state != AXIS_STATE_MOTOR_CALIBRATION) {
    Serial.printf("Invalid axis state: 0x%08X\n", axis_state);
    return;
  }

  switch (axis_state) {
    case AXIS_STATE_IDLE:
      // Stop the motor
      digitalWrite(MOTOR_INA_PIN, LOW);
      digitalWrite(MOTOR_INB_PIN, LOW);
      ledcWrite(LEDC_CHANNEL_0, 0);
      Serial.println("Axis state set to IDLE.");
      break;

    case AXIS_STATE_CLOSED_LOOP:
      // Initialize closed-loop control logic
      // TODO: Implement closed-loop control logic
      Serial.println("Axis state set to CLOSED-LOOP CONTROL.");
      break;

    case AXIS_STATE_MOTOR_CALIBRATION:
      // Initialize motor calibration logic
      // TODO: Implement motor calibration logic
      Serial.println("Axis state set to MOTOR CALIBRATION.");
      break;

    default:
      Serial.printf("Unknown axis state: 0x%08X\n", axis_state);
      break;
  }

  // Update the global state variable
  currentAxisState = axis_state;
}

void SetInputPosition(uint8_t* data) {
  Serial.println("Handling Set Input Position...");
  float position = *((float*)data);
  float velocity_feedforward = *((float*)(data + 4));
  float torque_feedforward = *((float*)(data + 8));
  // TODO: Implement logic to set input position
}

// Calculate the current velocity based on changes in position over time
float getVelocity() {
  static float lastPosition = 0;
  static unsigned long lastTime = 0;
  float currentPosition = readAS5600();
  
  // Handle I2C error
  if (currentPosition == -1) {
    return 0; // Return 0 velocity on error
  }

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

  // Handle wraparound
  float diff = currentPosition - lastPosition;
  if (diff > 180.0) {
    diff -= 360.0;
  } else if (diff < -180.0) {
    diff += 360.0;
  }

  float velocity = diff / deltaTime;
  lastPosition = currentPosition;
  lastTime = currentTime;

  return velocity;
}

// Handle the Set Input Velocity message
void SetInputVelocity(uint8_t* data) {
  Serial.println("Handling Set Input Velocity...");
  float desiredVelocity = *((float*)data); // Desired velocity in degrees per second
  float torqueFeedforward = *((float*)(data + 4));

  Serial.printf("Desired Velocity: %f, Torque Feedforward: %f\n", desiredVelocity, torqueFeedforward);

  // Ensure the motor is in the correct state
  if (currentAxisState != AXIS_STATE_CLOSED_LOOP) {
    Serial.println("Motor is not in closed-loop control state.");
    return;
  }

  // Calculate the current velocity in degrees per second
  float currentVelocity = getVelocity();
  float error = desiredVelocity - currentVelocity;

  // Implement a simple proportional control for demonstration
  float controlSignal = kp * error + torqueFeedforward;

  // Range check for controlSignal
  if (controlSignal > 255) {
    controlSignal = 255;
  } else if (controlSignal < -255) {
    controlSignal = -255;
  }

  // Apply the control signal to the motor
  controlMotor(controlSignal);

  // Log the calculated control signal and current velocity
  Serial.printf("Control Signal: %f, Current Velocity: %f\n", controlSignal, currentVelocity);
}

void SetInputTorque(uint8_t* data) {
  Serial.println("Handling Set Input Torque...");
  float torque = *((float*)data);
  // TODO: Implement logic to set input torque
}

void SetLimits(uint8_t* data) {
  Serial.println("Handling Set Limits...");
  float velocity_limit = *((float*)data);
  float current_limit = *((float*)(data + 4));
  // TODO: Implement logic to set limits
}

void SetControllerModes(uint8_t* data) {
  Serial.println("Handling Set Controller Modes...");
  uint32_t control_mode = *((uint32_t*)data);
  uint32_t input_mode = *((uint32_t*)(data + 4));
  // TODO: Implement logic to set controller modes
}

void SetTrajectoryParameters(uint8_t* data) {
  Serial.println("Handling Set Trajectory Parameters...");
  float accel_limit = *((float*)data);
  float decel_limit = *((float*)(data + 4));
  float traj_vel_limit = *((float*)(data + 8));
  // TODO: Implement logic to set trajectory parameters
}

void RequestEncoderEstimate(uint8_t* data) {
  Serial.println("Handling Request Encoder Estimate...");
  // TODO: Implement logic to request encoder estimate
}

void RequestErrorState(uint8_t* data) {
  Serial.println("Handling Request Error State...");
  // TODO: Implement logic to request error state
}

void ClearErrors(uint8_t* data) {
  Serial.println("Handling Clear Errors...");
  // TODO: Implement logic to clear errors
}

void controlMotor(float pidOutput) {
  if (pidOutput == 0) {
    // Ensure motors are off if no PID output
    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, LOW);
    ledcWrite(LEDC_CHANNEL_0, 0);
    return;
  }

  int pwmValue = abs((int)pidOutput); // Get the absolute value of the PID output for PWM

  // Ensure pwmValue does not exceed the maximum allowable PWM value
  if (pwmValue > 255) {
    pwmValue = 255;
  }

  if (pidOutput > 0) {
    digitalWrite(MOTOR_INA_PIN, HIGH);
    digitalWrite(MOTOR_INB_PIN, LOW);
    ledcWrite(LEDC_CHANNEL_0, pwmValue); // Forward PWM
  } else {
    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, HIGH);
    ledcWrite(LEDC_CHANNEL_0, pwmValue); // Backward PWM
  }
}

float readAS5600() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_REGISTER); // Start reading from the angle register
  if (Wire.endTransmission(false) != 0) {
    Serial.println("I2C communication failed!");
    return -1; // Return an error value
  }

  Wire.requestFrom(AS5600_ADDRESS, 2);
  if (Wire.available() < 2) {
    Serial.println("I2C data unavailable!");
    return -1; // Return an error value
  }

  int rawAngle = (Wire.read() << 8) | Wire.read();
  float angleInDegrees = (rawAngle * 360.0) / 4096.0; // Convert raw 12-bit value to degrees
  return angleInDegrees;
}
