#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// I2C pin definitions
#define SDA_PIN 9
#define SCL_PIN 10

// OLED display width and height, in pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// I2C address for the SSD1306 display
#define SSD1306_I2C_ADDRESS 0x3D

// Motor control pin definitions
#define MOTOR_PWM_PIN 3
#define MOTOR_INA_PIN 20
#define MOTOR_INB_PIN 8

// LEDC channel and timer definitions
#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_0 0
#define LEDC_BASE_FREQ 12000 // 12 kHz

// Create an instance of the SSD1306 display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Add a delay to ensure the display is ready
    delay(1000);

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialize the display
    if (!display.begin(SSD1306_I2C_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
    }

    // Clear the buffer
    display.clearDisplay();

    // Set text size and color
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text

    // Set cursor position
    display.setCursor(10, 25);   // (x, y) position

    // Display static text
    display.println(F("RoboDawg"));

    // Display the buffer
    display.display();

    // Initialize motor control pins
    pinMode(MOTOR_INA_PIN, OUTPUT);
    pinMode(MOTOR_INB_PIN, OUTPUT);

    // Configure LEDC PWM for motor control
    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, 8); // 8-bit resolution
    ledcAttachPin(MOTOR_PWM_PIN, LEDC_CHANNEL_0); // Attach the PWM pin to the channel
}

void loop() {
    // Drive motor forwards at 25% PWM
    digitalWrite(MOTOR_INA_PIN, HIGH);
    digitalWrite(MOTOR_INB_PIN, LOW);
    ledcWrite(LEDC_CHANNEL_0, 64); // 25% of 255
    delay(1000); // Drive for 1 second

    // Drive motor backwards at 25% PWM
    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, HIGH);
    ledcWrite(LEDC_CHANNEL_0, 64); // 25% of 255
    delay(1000); // Drive for 1 second
}
