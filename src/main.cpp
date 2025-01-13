#include <Arduino.h>
#include <Wire.h>        // I2C library
#include <driver/twai.h> // ESP32 TWAI driver for CAN communication
#include <EEPROM.h>
#include <PID_v1.h> // PID library

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

// CANbus/TWAI address
//#define DEVICE_TWAI_ID 0x123 // Ankle board address
//#define DEVICE_TWAI_ID 0x125 // Knee board address
#define DEVICE_TWAI_ID 0x127 // Hip board address

float kp = 10.0; // Default proportional gain
float ki = 0.5;  // Default integral gain
float kd = 0.1;  // Default derivative gain

#define EEPROM_KP_ADDR 8           // EEPROM address for kp
#define EEPROM_KI_ADDR 12          // EEPROM address for ki
#define EEPROM_KD_ADDR 16          // EEPROM address for kd
#define EEPROM_MARKER_ADDR 20      // EEPROM address for marker
#define EEPROM_ZERO_MARKER_ADDR 21 // EEPROM address for zero marker
#define EEPROM_DIRECTION_ADDR 24   // EEPROM address for direction

float targetAngle = 0.0;
float currentAngle = 0.0;
float lastAdjustedAngle = 0.0;
float totalAngle = 0.0;
int endPoint = 0;   // Endpoint determined after zeroing
int zeroOffset = 0; // Offset determined during manual zeroing
float deadband = 0.0;
int endpointSafezone = 5;
float pidOutput;
float motorCurrent = 0;

// PID variables
double setpoint, input, output;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// EEPROM addresses for storing zero and endpoint
#define EEPROM_ZERO_ADDR 0
#define EEPROM_END_ADDR 4

// System state
bool isZeroed = false; // Flag to track if the system has been zeroed
int direction = 1;     // Direction determined during zeroing
bool flipOutput = false; // Flag to track if the PID output should be flipped

// LEDC channel and timer definitions
#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_0 0
#define LEDC_BASE_FREQ 12000 // 16 kHz

void setupPins();
float readAngle();
void controlMotor(float pidOutput);
void receiveSerialInput();
void printLiveData(float currentAngle, float targetAngle, float pidOutput, float kp, float ki, float kd);
void saveToEEPROM();
void loadFromEEPROM();
void performZeroing();
void sendLiveDataTWAI(float currentAngle, int motorCurrent, int pwm);
void receiveTWAIInput();

void setup()
{
    setupPins();
    Serial.begin(115200);
    Serial.println("System initialized.");

    EEPROM.begin(512); // Initialize EEPROM with 512 bytes
    loadFromEEPROM();

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("I2C initialized.");

    // TWAI configuration
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        Serial.println("TWAI Driver installed.");
    }
    else
    {
        Serial.println("TWAI Driver installation failed.");
        while (1)
            ;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        Serial.println("TWAI Driver started.");
    }
    else
    {
        Serial.println("Failed to start TWAI Driver.");
        while (1)
            ;
    }

    // Initialize PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255);
    myPID.SetSampleTime(10);              // Sample time in milliseconds
    myPID.SetTunings(kp, ki, kd, P_ON_E); // Proportional on Error

    // Configure LEDC PWM for motor control
    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, 8); // 8-bit resolution
    ledcAttachPin(MOTOR_PWM_PIN, LEDC_CHANNEL_0); // Attach the PWM pin to the channel
}

void loop()
{
    receiveSerialInput(); // Check for serial input
    receiveTWAIInput();   // Check for TWAI input

    currentAngle = readAngle();
    input = currentAngle;
    setpoint = targetAngle;

    if (isZeroed && targetAngle != 0)
    {
        // Compute PID output
        myPID.Compute();
        // Apply deadband
        if (abs(setpoint - input) < deadband)
        {
            output = 0;
        }
        pidOutput = output;

        controlMotor(pidOutput);
    }
    else
    {
        controlMotor(0); // Ensure motors stay off if not zeroed or no target angle
    }

    // Update the display with live data
    printLiveData(currentAngle, targetAngle, pidOutput, kp, ki, kd);

    delay(10); // Loop delay for stability
}

void setupPins()
{
    pinMode(CURRENT_SENSOR_PIN, INPUT);
    pinMode(MOTOR_INA_PIN, OUTPUT);
    pinMode(MOTOR_INB_PIN, OUTPUT);
}

float readAngle()
{
    // Start I2C transmission to the AS5600 encoder
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(AS5600_ANGLE_REGISTER); // Start reading from the angle register
    if (Wire.endTransmission(false) != 0)
    { // Repeated start
        Serial.println("I2C communication failed!");
        return currentAngle; // Return last valid angle
    }

    // Request 2 bytes from the AS5600 encoder
    Wire.requestFrom(AS5600_ADDRESS, 2);
    if (Wire.available() < 2)
    {
        Serial.println("I2C data unavailable!");
        return currentAngle; // Return last valid angle
    }

    // Combine high and low bytes into a raw 12-bit angle value
    int rawAngle = (Wire.read() << 8) | Wire.read();

    // Convert raw 12-bit angle to degrees
    float angleInDegrees = (rawAngle * 360.0) / 4096.0;

    // Apply zero offset and direction
    float adjustedAngle = (angleInDegrees - zeroOffset) * direction;

    

    // Handle wrap-around
    float diff = adjustedAngle - lastAdjustedAngle;

    if (diff > 180.0)
    {
        diff -= 360.0;
    }
    else if (diff < -180.0)
    {
        diff += 360.0;
    }

    totalAngle += diff;
    lastAdjustedAngle = adjustedAngle;

    
    // Debugging info
    // Serial.print("Raw Angle: "); Serial.print(rawAngle);
    // Serial.print(" | Adjusted Angle: "); Serial.print(adjustedAngle);
    // Serial.print(" | Total Angle: "); Serial.println(totalAngle);
    
    totalAngle = abs(totalAngle); // Ensure total angle is always positive
    return totalAngle;
}

void controlMotor(float pidOutput)
{
    if (flipOutput)
    {
        pidOutput = -pidOutput; // Flip the PID output if the flag is set
    }

    int pwmValue = (int)abs(pidOutput); // Get the absolute value of the PID output for PWM

    if (pidOutput == 0)
    {
        // Ensure motors are off if no PID output
        digitalWrite(MOTOR_INA_PIN, LOW);
        digitalWrite(MOTOR_INB_PIN, LOW);
        ledcWrite(LEDC_CHANNEL_0, 0);
        return;
    }

    if (pidOutput > 0)
    {
        digitalWrite(MOTOR_INA_PIN, HIGH);
        digitalWrite(MOTOR_INB_PIN, LOW);
        ledcWrite(LEDC_CHANNEL_0, pwmValue); // Forward PWM
    }
    else
    {
        digitalWrite(MOTOR_INA_PIN, LOW);
        digitalWrite(MOTOR_INB_PIN, HIGH);
        ledcWrite(LEDC_CHANNEL_0, pwmValue); // Backward PWM
    }
}

void receiveSerialInput()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove any trailing whitespace

        if (input.equalsIgnoreCase("ZERO"))
        {
            performZeroing();
        }
        else if (input.equalsIgnoreCase("FLIP"))
        {
            flipOutput = !flipOutput; // Toggle the flip flag
            Serial.print("PID output flipped: ");
            Serial.println(flipOutput ? "ON" : "OFF");
        }
        else if (input.startsWith("P="))
        {
            kp = input.substring(2).toFloat();
            myPID.SetTunings(kp, ki, kd);
            saveToEEPROM();
            Serial.print("Updated Kp and saved to EEPROM: ");
            Serial.println(kp);
        }
        else if (input.startsWith("I="))
        {
            ki = input.substring(2).toFloat();
            myPID.SetTunings(kp, ki, kd);
            saveToEEPROM();
            Serial.print("Updated Ki and saved to EEPROM: ");
            Serial.println(ki);
        }
        else if (input.startsWith("D="))
        {
            kd = input.substring(2).toFloat();
            myPID.SetTunings(kp, ki, kd);
            saveToEEPROM();
            Serial.print("Updated Kd and saved to EEPROM: ");
            Serial.println(kd);
        }
        else if (input.toFloat() >= 0.0 && input.toFloat() <= 360.0)
        {
            if (!isZeroed)
            {
                Serial.println("System is not zeroed. Type 'ZERO' to set zero offset.");
                return;
            }

            targetAngle = input.toFloat();
            Serial.print("New target angle: ");
            Serial.println(targetAngle);
        }
        else
        {
            Serial.println("Invalid input. Use 'ZERO', 'FLIP', a number (0-360) for target angle, or 'P=value', 'I=value', 'D=value' to set PID parameters.");
        }
    }
}

void performZeroing()
{
    Serial.println("Zeroing process started...");
    zeroOffset = 0;
    float initialAngle = readAngle();
    direction = initialAngle > 180 ? -1 : 1; // Determine direction to decrease the angle

    // Drive motor at 25% PWM for 2 seconds to find zero
    int pwmValue = 200; // 25% of 255
    if (direction == 1)
    {
        digitalWrite(MOTOR_INA_PIN, LOW);
        digitalWrite(MOTOR_INB_PIN, HIGH);
        ledcWrite(LEDC_CHANNEL_0, pwmValue);
    }
    else
    {
        digitalWrite(MOTOR_INA_PIN, HIGH);
        digitalWrite(MOTOR_INB_PIN, LOW);
        ledcWrite(LEDC_CHANNEL_0, pwmValue);
    }
    delay(2000); // Drive motor for 2 seconds

    // Set zero offset
    zeroOffset = readAngle();

    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, LOW);
    ledcWrite(LEDC_CHANNEL_0, 0);

    // Find endpoint by driving in the opposite direction
    Serial.println("Finding endpoint...");
    direction *= -1;
    float maxAngle = zeroOffset;
    unsigned long startTime = millis();
    if (direction == 1)
    {
        digitalWrite(MOTOR_INA_PIN, LOW);
        digitalWrite(MOTOR_INB_PIN, HIGH);
        ledcWrite(LEDC_CHANNEL_0, pwmValue);
    }
    else
    {
        digitalWrite(MOTOR_INA_PIN, HIGH);
        digitalWrite(MOTOR_INB_PIN, LOW);
        ledcWrite(LEDC_CHANNEL_0, pwmValue);
    }

    while (millis() - startTime < 2000)
    {
        float currentAngle = readAngle();
        if (currentAngle > maxAngle)
        {
            maxAngle = currentAngle;
        }
    }

    // Record endpoint
    endPoint = maxAngle;

    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, LOW);
    ledcWrite(LEDC_CHANNEL_0, 0);

    // Save direction to EEPROM
    EEPROM.put(EEPROM_DIRECTION_ADDR, direction);
    EEPROM.commit();

    // Save to EEPROM
    saveToEEPROM();

    isZeroed = true;
    Serial.println("Zeroing complete. Zero and endpoint updated.");
}

void printLiveData(float currentAngle, float targetAngle, float pidOutput, float kp, float ki, float kd)
{
    Serial.print("\033[2K"); // Clear the current line
    Serial.print("\r");      // Move the cursor to the beginning of the line

    Serial.print("Target: ");
    Serial.print(targetAngle);
    Serial.print(" | Current Angle: ");
    Serial.print(abs(totalAngle)); // Ensure the current angle is displayed as positive
    Serial.print(" | PID Output: ");
    Serial.print(pidOutput);
    Serial.print(" | Direction: ");
    if (pidOutput > 0)
    {
        Serial.print("Forward");
    }
    else if (pidOutput < 0)
    {
        Serial.print("Reverse");
    }
    else
    {
        Serial.print("Stopped");
    }
    Serial.print(" | PWM: ");
    Serial.print(abs((int)pidOutput)); // Absolute value of PID output for PWM
    Serial.print(" | Kp: ");
    Serial.print(kp);
    Serial.print(" | Ki: ");
    Serial.print(ki);
    Serial.print(" | Kd: ");
    Serial.print(kd);

    Serial.print(" |");
}

void saveToEEPROM()
{
    EEPROM.put(EEPROM_ZERO_ADDR, zeroOffset);
    EEPROM.put(EEPROM_END_ADDR, endPoint);
    EEPROM.put(EEPROM_KP_ADDR, kp);
    EEPROM.put(EEPROM_KI_ADDR, ki);
    EEPROM.put(EEPROM_KD_ADDR, kd);
    EEPROM.put(EEPROM_DIRECTION_ADDR, direction);

    // Set EEPROM markers to indicate that values have been initialized
    byte marker = 1;
    EEPROM.put(EEPROM_MARKER_ADDR, marker);
    EEPROM.put(EEPROM_ZERO_MARKER_ADDR, marker);

    EEPROM.commit();
    Serial.println("Values saved to EEPROM.");
}

void loadFromEEPROM()
{
    byte marker;
    EEPROM.get(EEPROM_MARKER_ADDR, marker);
    byte zeroMarker;
    EEPROM.get(EEPROM_ZERO_MARKER_ADDR, zeroMarker);

    if (marker != 1)
    {
        // If marker is not set, initialize with default PID values and save to EEPROM
        kp = 1.0;
        ki = 0.0;
        kd = 0.0;
        saveToEEPROM();
        Serial.println("EEPROM marker not found. Initialized with default PID values.");
    }
    else
    {
        // Load PID values from EEPROM
        EEPROM.get(EEPROM_KP_ADDR, kp);
        EEPROM.get(EEPROM_KI_ADDR, ki);
        EEPROM.get(EEPROM_KD_ADDR, kd);
        Serial.println("PID values loaded from EEPROM.");
    }

    if (zeroMarker == 1)
    {
        // Load zero and endpoint values from EEPROM
        EEPROM.get(EEPROM_ZERO_ADDR, zeroOffset);
        EEPROM.get(EEPROM_END_ADDR, endPoint);
        isZeroed = true;
        Serial.println("Zero and endpoint values loaded from EEPROM.");
    }
    else
    {
        isZeroed = false;
        Serial.println("System is not zeroed. Please perform zeroing.");
    }

    // Load direction from EEPROM
    EEPROM.get(EEPROM_DIRECTION_ADDR, direction);

    // Initialize PID with loaded values
    myPID.SetTunings(kp, ki, kd);
}

void sendLiveDataTWAI(float currentAngle, int motorCurrent, int pwm)
{
    twai_message_t message;
    message.identifier = DEVICE_TWAI_ID;
    message.extd = 0;             // Standard frame
    message.rtr = 0;              // Data frame
    message.data_length_code = 6; // Up to 6 bytes

    // Pack data into the message payload
    int angle = (int)currentAngle;
    message.data[0] = (uint8_t)(angle & 0xFF);
    message.data[1] = (uint8_t)((angle >> 8) & 0xFF);
    message.data[2] = (uint8_t)(motorCurrent & 0xFF);  // Motor current instead of direction
    message.data[3] = (uint8_t)(pwm & 0xFF);
    message.data[4] = (uint8_t)((pwm >> 8) & 0xFF);
    message.data[5] = 0; // Reserved for future use or additional flags

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK)
    {
        //Serial.println("Live data sent over TWAI.");
    }
    else
    {
        Serial.println("Failed to send live data over TWAI.");
    }
}

void receiveTWAIInput()
{
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        // Print the entire TWAI message
        Serial.print("Received TWAI message: ID=");
        Serial.print(message.identifier, HEX);
        Serial.print(" DLC=");
        Serial.print(message.data_length_code);
        Serial.print(" Data=");
        for (int i = 0; i < message.data_length_code; i++)
        {
            Serial.print(message.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        if (message.data_length_code == 2 && message.identifier == DEVICE_TWAI_ID)
        {
            // Handle target angle sent by the master
            int targetAngle = (message.data[1] << 8) | message.data[0];
            if (targetAngle >= 0 && targetAngle <= 360)
            {
                if (!isZeroed)
                {
                    Serial.println("System is not zeroed. Type 'ZERO' to set zero offset via TWAI.");
                    return;
                }

                // Update the target angle
                ::targetAngle = targetAngle; // Use the global variable or the appropriate scope
                //Serial.print("New target angle set via TWAI: ");
                //Serial.println(::targetAngle); // Use the global variable or the appropriate scope

                // Send data back to the master
                const char *direction = (pidOutput > 0) ? "Forward" : (pidOutput < 0) ? "Reverse"
                                                                                      : "Stopped";
                sendLiveDataTWAI(currentAngle, motorCurrent, abs((int)pidOutput)); // Use the global variable or the appropriate scope
            }
            else
            {
                Serial.println("Invalid target angle received via TWAI.");
            }
        }
        else
        {
            // Handle other types of messages as strings
            String input;
            for (int i = 0; i < message.data_length_code; i++)
            {
                input += (char)message.data[i];
            }

            input.trim(); // Remove any trailing whitespace

            if (input.equalsIgnoreCase("ZERO"))
            {
                performZeroing();
            }
            else if (input.equalsIgnoreCase("FLIP"))
            {
                flipOutput = !flipOutput; // Toggle the flip flag
                Serial.print("PID output flipped via TWAI: ");
                Serial.println(flipOutput ? "ON" : "OFF");
            }
            else if (input.startsWith("P="))
            {
                kp = input.substring(2).toFloat();
                myPID.SetTunings(kp, ki, kd);
                saveToEEPROM();
                Serial.print("Updated Kp via TWAI and saved to EEPROM: ");
                Serial.println(kp);
            }
            else if (input.startsWith("I="))
            {
                ki = input.substring(2).toFloat();
                myPID.SetTunings(kp, ki, kd);
                saveToEEPROM();
                Serial.print("Updated Ki via TWAI and saved to EEPROM: ");
                Serial.println(ki);
            }
            else if (input.startsWith("D="))
            {
                kd = input.substring(2).toFloat();
                myPID.SetTunings(kp, ki, kd);
                saveToEEPROM();
                Serial.print("Updated Kd via TWAI and saved to EEPROM: ");
                Serial.println(kd);
            }
            else
            {
                Serial.println("Invalid TWAI input. Use 'ZERO', 'FLIP', a number (0-360) for target angle, or 'P=value', 'I=value', 'D=value' to set PID parameters.");
            }
        }
    }
}
