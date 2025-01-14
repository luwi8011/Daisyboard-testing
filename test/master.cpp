#include <Arduino.h>
#include <driver/twai.h>

// TWAI configuration
#define CAN_RX_PIN 2
#define CAN_TX_PIN 1
#define MASTER_TWAI_ID 0x100

#define ANKLE_TWAI_ID 0x123 // Ankle board address
#define KNEE_TWAI_ID 0x125  // Knee board address
#define HIP_TWAI_ID 0x127   // Hip board address

// Target angle variables for each joint
#define MAX_JOINTS 3
int targetAngle[MAX_JOINTS] = {0, 0, 0};
unsigned long lastSendTime = 0; // Variable to track the last send time

struct JointData {
    float currentAngle;
    int motorCurrent;
    int pwm;
};

JointData jointData[MAX_JOINTS];

void setupTWAI();
void relaySerialToTWAI();
void sendTargetAngleToSlaves();
void handleSlaveData();
void printLiveData();

void setup()
{
    Serial.begin(115200);
    Serial.println("Master system initialized.");
    setupTWAI();
}

void loop()
{
    relaySerialToTWAI();
    handleSlaveData();

    // Send the target angles to the slaves every 20 milliseconds
    if (millis() - lastSendTime >= 20)
    {
        sendTargetAngleToSlaves();
        lastSendTime = millis();
    }
}

void setupTWAI()
{
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
}

void relaySerialToTWAI()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove any trailing whitespace

        // Split the input into parts based on the ':' delimiter
        int firstColon = input.indexOf(':');
        int secondColon = input.indexOf(':', firstColon + 1);

        if (firstColon == -1 || secondColon == -1)
        {
            Serial.println("Invalid input format. Use <COMMAND>:<JOINT_ADDRESS>:<DATA>");
            return;
        }

        String command = input.substring(0, firstColon);
        String jointAddressStr = input.substring(firstColon + 1, secondColon);
        String data = input.substring(secondColon + 1);

        int jointAddress = strtol(jointAddressStr.c_str(), NULL, 16);

        if (jointAddress != HIP_TWAI_ID && jointAddress != KNEE_TWAI_ID && jointAddress != ANKLE_TWAI_ID)
        {
            Serial.println("Invalid joint address. Use 0x127, 0x125, or 0x123.");
            return;
        }

        if (command.equalsIgnoreCase("FLIP"))
        {
            // Send FLIP command to the specified joint
            twai_message_t message;
            message.identifier = jointAddress;
            message.extd = 0;             // Standard frame
            message.rtr = 0;              // Data frame
            message.data_length_code = 4; // Sending the FLIP command
            message.data[0] = 'F';
            message.data[1] = 'L';
            message.data[2] = 'I';
            message.data[3] = 'P';
            if (twai_transmit(&message, pdMS_TO_TICKS(10)) != ESP_OK)
            {
                Serial.print("Failed to send FLIP command to ");
                Serial.println(jointAddressStr);
            }
            else
            {
                Serial.print("FLIP command sent to ");
                Serial.print(jointAddressStr);
                Serial.println(" via TWAI.");
            }
        }
        else if (command.equalsIgnoreCase("SET_ANGLE"))
        {
            int angle = data.toInt();
            if (angle >= 0 && angle <= 360)
            {
                if (jointAddress == HIP_TWAI_ID)
                {
                    targetAngle[0] = angle;
                    Serial.print("New target angle for HIP set via Serial: ");
                    Serial.println(targetAngle[0]);
                }
                else if (jointAddress == KNEE_TWAI_ID)
                {
                    targetAngle[1] = angle;
                    Serial.print("New target angle for KNEE set via Serial: ");
                    Serial.println(targetAngle[1]);
                }
                else if (jointAddress == ANKLE_TWAI_ID)
                {
                    targetAngle[2] = angle;
                    Serial.print("New target angle for ANKLE set via Serial: ");
                    Serial.println(targetAngle[2]);
                }
            }
            else
            {
                Serial.println("Invalid angle. Please enter a number between 0 and 360.");
            }
        }
        else if (command.equalsIgnoreCase("ZERO"))
        {
            // Send ZERO command to the specified joint
            twai_message_t message;
            message.identifier = jointAddress;
            message.extd = 0;             // Standard frame
            message.rtr = 0;              // Data frame
            message.data_length_code = 4; // Sending the ZERO command
            message.data[0] = 'Z';
            message.data[1] = 'E';
            message.data[2] = 'R';
            message.data[3] = 'O';
            if (twai_transmit(&message, pdMS_TO_TICKS(10)) != ESP_OK)
            {
                Serial.print("Failed to send ZERO command to ");
                Serial.println(jointAddressStr);
            }
            else
            {
                Serial.print("ZERO command sent to ");
                Serial.print(jointAddressStr);
                Serial.println(" via TWAI.");
            }
        }
        else
        {
            Serial.println("Invalid command. Use FLIP, SET_ANGLE, or ZERO.");
        }
    }
}

void sendTargetAngleToSlaves()
{
    twai_message_t message;
    message.extd = 0;             // Standard frame
    message.rtr = 0;              // Data frame
    message.data_length_code = 2; // Sending only the target angle (2 bytes)

    // Pack the target angle into the message payload and send to each joint
    int jointAddresses[MAX_JOINTS] = {HIP_TWAI_ID, KNEE_TWAI_ID, ANKLE_TWAI_ID};
    for (int i = 0; i < MAX_JOINTS; i++)
    {
        message.identifier = jointAddresses[i]; // Target joint ID
        message.data[0] = (uint8_t)(targetAngle[i] & 0xFF);
        message.data[1] = (uint8_t)((targetAngle[i] >> 8) & 0xFF);
        if (twai_transmit(&message, pdMS_TO_TICKS(10)) != ESP_OK)
        {
            Serial.print("Failed to send target angle to joint ");
            Serial.println(jointAddresses[i], HEX);
        }
    }
}

void handleSlaveData()
{
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        if (message.data_length_code != 6)
        {
            return;
        }

        // Parse the incoming message
        float currentAngle = (message.data[1] << 8) | message.data[0];
        int motorCurrent = message.data[2]; // Read motor current from byte 2
        int pwm = (message.data[4] << 8) | message.data[3];

        // Store the parsed data
        const char* jointName;
        if (message.identifier == HIP_TWAI_ID)
        {
            jointData[0] = {currentAngle, motorCurrent, pwm};
            jointName = "HIP";
        }
        else if (message.identifier == KNEE_TWAI_ID)
        {
            jointData[1] = {currentAngle, motorCurrent, pwm};
            jointName = "KNEE";
        }
        else if (message.identifier == ANKLE_TWAI_ID)
        {
            jointData[2] = {currentAngle, motorCurrent, pwm};
            jointName = "ANKLE";
        }
        else
        {
            return;
        }

        

        // Print the stored data
        printLiveData();
    }
}

void printLiveData()
{
    for (int i = 0; i < MAX_JOINTS; i++)
    {
        const char* jointName = (i == 0) ? "HIP" : (i == 1) ? "KNEE" : "ANKLE";
        Serial.print(jointName);
        Serial.print(":angle=");
        Serial.print(jointData[i].currentAngle, 2); // Print with 2 decimal places
        Serial.print(",current=");
        Serial.println(jointData[i].motorCurrent);
    }
}
