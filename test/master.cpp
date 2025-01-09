#include <Arduino.h>
#include <driver/twai.h>

// TWAI configuration
#define CAN_RX_PIN 2
#define CAN_TX_PIN 1
#define MASTER_TWAI_ID 0x100

const int SLAVE_TWAI_ID[2] = {0x123, 0x125};
// Target angle variable
#define MAXANGLES 2
int targetAngle[MAXANGLES] = {0, 0};
unsigned long lastSendTime = 0; // Variable to track the last send time

void setupTWAI();
void relaySerialToTWAI();
void sendTargetAngleToSlave();
void handleSlaveData();
void printLiveData(int target, int current, const char *direction, int pwm);

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

  // Send the target angle to the slave every 20 milliseconds
  if (millis() - lastSendTime >= 20)
  {
    sendTargetAngleToSlave();
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
    int angle[MAXANGLES];
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any trailing whitespace
    Serial.print(input);
    char lastChar = input.charAt(input.length() - 1);
    input.remove(input.length() - 1);

    // Update the target angle based on serial input
    if (lastChar == 'a')
    {
      angle[0] = input.toInt();
    }
    else if (lastChar == 'b')
    {
      angle[1] = input.toInt();
    }
    for (int i = 0; i < MAXANGLES; i++)
    {
      if (angle[i] >= 0 && angle[i] <= 360)
      {
        targetAngle[i] = angle[i];
        Serial.print("New target angle ");
        Serial.print(i);
        Serial.print(" set via Serial: ");
        Serial.println(targetAngle[i]);
      }
      else
      {
        Serial.println("Invalid input. Please enter a number between 0 and 360.");
      }
    }
  }
}

void sendTargetAngleToSlave()
{
  twai_message_t message;
  message.extd = 0;             // Standard frame
  message.rtr = 0;              // Data frame
  message.data_length_code = 2; // Sending only the target angle (2 bytes)

  // Pack the target angle into the message payload
  for (int i = 0; i < MAXANGLES; i++)
  {
    message.identifier = SLAVE_TWAI_ID[i]; // Target slave ID
    message.data[0] = (uint8_t)(targetAngle[i] & 0xFF);
    message.data[1] = (uint8_t)((targetAngle[i] >> 8) & 0xFF);
    twai_transmit(&message, pdMS_TO_TICKS(10));
  }

  
}

void handleSlaveData()
{
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    // Ensure the message data is within the expected length
    if (message.data_length_code != 8)
    {
      return;
    }

    // Parse the incoming message
    int target = (message.data[1] << 8) | message.data[0];
    int current = (message.data[3] << 8) | message.data[2];
    const char *direction = (message.data[4] == 1) ? "Forward" : (message.data[4] == 2) ? "Reverse"
                                                                                        : "Unknown";
    int pwm = (message.data[6] << 8) | message.data[5];

    // Print the parsed data
    printLiveData(target, current, direction, pwm);
  }
}

void printLiveData(int target, int current, const char *direction, int pwm)
{
  Serial.print("\r"); // Move the cursor to the beginning of the line
  Serial.print("Slave Data - ");
  Serial.print("Target: ");
  Serial.print(target);
  Serial.print(", Current: ");
  Serial.print(current);
  Serial.print(", Direction: ");
  Serial.print(direction);
  Serial.print(", PWM: ");
  Serial.print(pwm);
  Serial.print("   "); // Additional spaces to clear any leftover characters from previous prints
  Serial.print(", Master Target: ");
  Serial.print(targetAngle[0]);
  Serial.print("   "); // Additional spaces to clear any leftover characters from previous prints
}