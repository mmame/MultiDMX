#include <Arduino.h>
#include <esp_dmx.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include "display.h"
#include <Wire.h>
#include "webpage.h"
#include "SoftwareSerial.h"

// ✅ DMX RS-485 Pins
#define PIN_RS485_TX   17
#define PIN_RS485_RX   16
#define PIN_RS485_EN   21 

// ✅ DIP Switch (74HC165) Pins
#define PIN_DIP_CLK    22 
#define PIN_DIP_LATCH  25 
#define PIN_DIP_DATA   34  // Input Only

// ✅ Stepper Control Pins
#define PIN_TMC2209_UART  14  // UART for TMC2209
#define PIN_TMC2209_DIR   15  // DIR signal (direction)
#define PIN_TMC2209_STEP  32  // STEP signal (pulses)

// ✅ H-Bridge Motor Pins
#define PIN_MOTOR_A_1  4
#define PIN_MOTOR_A_2  5
#define PIN_MOTOR_B_1  18
#define PIN_MOTOR_B_2  19

// ✅ Servo Control Pins
#define PIN_SERVO_1    23
#define PIN_SERVO_2    13
#define PIN_SERVO_3    2
#define PIN_SERVO_4    12

// ✅ Relay Control Pins
#define PIN_RELAY_1    27
#define PIN_RELAY_2    26

// ✅ I2C Display Pins
#define PIN_I2C_SDA    33  
#define PIN_I2C_SCL    0  

#define PIN_BUTTON     35

// ✅ Centralized DMX Base Addresses
#define DMX_SERVO_1             (baseDMX)
#define DMX_SERVO_2             (baseDMX + 1)
#define DMX_SERVO_3             (baseDMX + 2)
#define DMX_SERVO_4             (baseDMX + 3)
#define DMX_MOTOR_A             (baseDMX + 4)
#define DMX_MOTOR_B             (baseDMX + 5)
#define DMX_STEPPER_SPEED       (baseDMX + 6)
#define DMX_STEPPER_POSITION    (baseDMX + 7)
#define DMX_RELAY_1             (baseDMX + 8)
#define DMX_RELAY_2             (baseDMX + 9)
#define DMX_LAST                DMX_RELAY_2

#define BUTTON_DEBOUNCE_DELAY_MS 100

// Stepper direction (true = reversed, false = normal)
const bool STEPPER_REVERSED = false;

// Individual servo direction settings
const bool SERVO_1_REVERSED = false;
const bool SERVO_2_REVERSED = false;
const bool SERVO_3_REVERSED = false;
const bool SERVO_4_REVERSED = false;

Display oledDisplay(PIN_I2C_SDA, PIN_I2C_SCL);

// ✅ Global Variables
volatile int32_t stepperPosition = 0;  // Hardware-tracked step count
dmx_port_t dmxPort = DMX_NUM_1;
byte data[DMX_PACKET_SIZE];
bool dmxIsConnected = false;
unsigned long lastUpdate = millis();
uint16_t baseDMX = 1;  // Base DMX Address
volatile bool buttonPressed = false;  // Updated globally in readDIPSwitch()
unsigned long lastButtonPressTime = 0;  // Track last state change

const int Servo1MinMicros = DEFAULT_SERVO_MICROS_MIN;
const int Servo1MaxMicros = DEFAULT_SERVO_MICROS_MAX;
const int Servo2MinMicros = DEFAULT_SERVO_MICROS_MIN;
const int Servo2MaxMicros = DEFAULT_SERVO_MICROS_MAX;
const int Servo3MinMicros = DEFAULT_SERVO_MICROS_MIN;
const int Servo3MaxMicros = DEFAULT_SERVO_MICROS_MAX;
const int Servo4MinMicros = DEFAULT_SERVO_MICROS_MIN;
const int Servo4MaxMicros = DEFAULT_SERVO_MICROS_MAX;

// ✅ LEDC Configuration for H Bridge
#define PWM_FREQ       5000  // 5 kHz PWM Frequency
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1

// Servo objects
Servo servo1, servo2, servo3, servo4;

// Stepper Driver
// FastAccelStepper Instance
FastAccelStepperEngine engine;
FastAccelStepper *stepper = NULL;
TMC2209Stepper tmc2209(PIN_TMC2209_UART, PIN_TMC2209_UART, 0.11, 0);
int32_t stepperTargetPosition = 0;  // Target position
bool homingActive = false; 
unsigned long homingStartTime = 0;
bool homingFailed = false;
char macSuffix[5];  // 4 chars + null terminator
WebConfig webConfig;

// ✅ Read DIP Switch (74HC165) to Determine Base DMX Address and Button State
uint16_t readDIPSwitch() {
    uint16_t value = 0;
    digitalWrite(PIN_DIP_LATCH, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_DIP_LATCH, HIGH);

    for (int i = 15; i >= 0; i--) {  // **Reverse bit order**
        value |= (digitalRead(PIN_DIP_DATA) << i);
        digitalWrite(PIN_DIP_CLK, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_DIP_CLK, LOW);
    }

    //Serial.printf("Input Value: %u\n", value);
    return value & 0x01FF;  // Mask out DMX address
}

void stepperStartHoming() {
    if (!homingActive) {  
        Serial.println("🏠 Starting Stepper Homing...");
        homingActive = true;
        homingFailed = false;
        homingStartTime = millis();  // Start timeout tracking

        stepper->setCurrentPosition(0);
        stepper->setSpeedInHz(DEFAULT_STEPPER_HOMING_SPEED);  
        stepper->setAcceleration(500);  
        stepper->moveTo(-DEFAULT_STEPPER_HOMING_STEP_LIMIT);  // Move indefinitely in reverse
    } 
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setup...");

    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(macSuffix, sizeof(macSuffix), "%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);

    oledDisplay.begin();
    webConfig.begin();

    // --- Configure DIP Switch Pins ---
    pinMode(PIN_DIP_CLK, OUTPUT);
    pinMode(PIN_DIP_LATCH, OUTPUT);
    pinMode(PIN_DIP_DATA, INPUT);
    digitalWrite(PIN_DIP_CLK, LOW);
    digitalWrite(PIN_DIP_LATCH, HIGH);

    // --- Configure Motor Pins ---
    pinMode(PIN_MOTOR_A_1, OUTPUT);
    pinMode(PIN_MOTOR_A_2, OUTPUT);
    pinMode(PIN_MOTOR_B_1, OUTPUT);
    pinMode(PIN_MOTOR_B_2, OUTPUT);
    digitalWrite(PIN_MOTOR_A_1, LOW);
    digitalWrite(PIN_MOTOR_A_2, LOW);
    digitalWrite(PIN_MOTOR_B_1, LOW);
    digitalWrite(PIN_MOTOR_B_2, LOW);

    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_A_1, PWM_CHANNEL_A);
    ledcAttachPin(PIN_MOTOR_B_1, PWM_CHANNEL_B);

    pinMode(PIN_TMC2209_STEP, OUTPUT);
    pinMode(PIN_TMC2209_DIR, OUTPUT);

    pinMode(PIN_BUTTON, INPUT);

    // --- Read DIP Switch ---
    baseDMX = readDIPSwitch();
    Serial.printf("Base DMX Address: %d\n", baseDMX);

    // --- Configure TMC2209 ---
    pinMode(PIN_TMC2209_STEP, OUTPUT);
    pinMode(PIN_TMC2209_DIR, OUTPUT);

    //tmc2209Serial.begin(9600, SERIAL_8N1, PIN_TMC2209_UART, PIN_TMC2209_UART);
    tmc2209.begin();
    delay(1000);
    Serial.println("\n=== TMC2209 UART Communication Check ===");

    Serial.printf(
        "IFCNT: %d\n",
        tmc2209.IFCNT()
    );

    Serial.println("========================================");

    //disable microstepping -> full steps
    tmc2209.mres(0);

    // Enable internal sense resistors
    tmc2209.internal_Rsense(true);

    tmc2209.rms_current(DEFAULT_STEPPER_TMC2209_CURRENT);
    tmc2209.toff(0);
    tmc2209.pwm_autoscale(true);

    // Configure sensorless homing (StallGuard)
    tmc2209.TCOOLTHRS(0xFFFFF);
    tmc2209.SGTHRS(DEFAULT_STEPPER_STALL_THRESHOLD);

    engine.init();    
    stepper = engine.stepperConnectToPin(PIN_TMC2209_STEP);
    stepper->setDirectionPin(PIN_TMC2209_DIR);
    stepper->setSpeedInHz(DEFAULT_STEPPER_MAX_SPEED);
    stepper->setAcceleration(DEFAULT_STEPPER_ACCEL);    
    stepper->setCurrentPosition(0);  // Assume starting at position 0

    // --- Configure DMX ---
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    dmx_driver_install(dmxPort, &config, NULL, 0);
    dmx_set_pin(dmxPort, PIN_RS485_TX, PIN_RS485_RX, PIN_RS485_EN);

    // --- Initialize Servos ---
    servo1.attach(PIN_SERVO_1);
    servo2.attach(PIN_SERVO_2);
    servo3.attach(PIN_SERVO_3);
    servo4.attach(PIN_SERVO_4);

    // --- Initialize Relays ---
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_RELAY_2, OUTPUT);
    digitalWrite(PIN_RELAY_1, LOW);
    digitalWrite(PIN_RELAY_2, LOW);

    tmc2209.toff(4);

    stepperStartHoming();

    Serial.println("TMC2209 READY.");
}

void controlStepper() {
    int dmxSpeed = data[DMX_STEPPER_SPEED];  
    int dmxTarget = data[DMX_STEPPER_POSITION];

    long stepperMaxSpeed = map(dmxSpeed, 0, 255, 1, DEFAULT_STEPPER_MAX_SPEED);  
    stepper->setSpeedInHz(stepperMaxSpeed);

    int newTarget = dmxTarget * DEFAULT_STEPPER_SCALE;

    if (STEPPER_REVERSED) {
        newTarget = -newTarget;  // Reverse direction
    }

    if (255 == data[DMX_STEPPER_POSITION] && !homingActive) {  
        stepperStartHoming();
    } 
    else if (!homingActive && newTarget != stepperTargetPosition) {  
        stepperTargetPosition = newTarget;
        stepper->moveTo(stepperTargetPosition);
        Serial.printf("Moving to position: %d steps speed %d\n", stepperTargetPosition, stepperMaxSpeed);
    }
}

void controlMotor(int dmxValue, int pin_pwm, int pin_direction, int pwm_channel, int motorIndex) {
    static int lastDmxValue[2] = {-1, -1};  // Assuming 2 motors (adjust if needed)
    static int lastSpeed[2] = {-1, -1};
    static bool lastDirection[2] = {false, false};

    if (dmxValue == lastDmxValue[motorIndex]) {
        return; // No change, skip updates
    }
    
    lastDmxValue[motorIndex] = dmxValue;
    int speed = 0;
    bool direction = false;

    if (dmxValue == 0 || dmxValue == 128) {  // 🚦 Stop Motor
        speed = 0;
    } else if (dmxValue < 128) {  // 🔄 Reverse
        speed = map(dmxValue, 1, 127, 1, 255);  // Reverse speed (1-255)
        direction = true;
    } else {  // 🚀 Forward
        speed = map(dmxValue, 129, 255, 1, 255);  // Forward speed (1-255)
        direction = false;
    }

    if (speed != lastSpeed[motorIndex] || direction != lastDirection[motorIndex]) {
        digitalWrite(pin_direction, direction ? HIGH : LOW);
        ledcWrite(pwm_channel, speed);

        Serial.printf("Motor %d updated - PWM %d: DMX=%d, Speed=%d, Direction=%s\n", 
                      motorIndex, pwm_channel, dmxValue, speed, direction ? "Reverse" : "Forward");

        lastSpeed[motorIndex] = speed;
        lastDirection[motorIndex] = direction;
    }
}

void controlServo(Servo &servo, int dmxValue, int &lastDmxValue, int minMicros, int maxMicros, bool isReversed) {
    if (dmxValue == lastDmxValue) {
        return; // No change, skip updates
    }

    lastDmxValue = dmxValue;
    int pulseWidth = map(dmxValue, 0, 255, minMicros, maxMicros);

    if (isReversed) {
        pulseWidth = minMicros + maxMicros - pulseWidth;  // Reverse direction
    }

    servo.writeMicroseconds(pulseWidth);

    Serial.printf("Servo updated - DMX=%d, Pulse Width=%d, Reversed=%s\n", dmxValue, pulseWidth, isReversed ? "Yes" : "No");
}

void loop() {
    dmx_packet_t packet;
    static int lastServoValues[4] = {-1, -1, -1, -1};

    baseDMX = readDIPSwitch();

    /*if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
        if (!packet.err) {
            dmxIsConnected = true;
            dmx_read(dmxPort, data, packet.size);
            controlMotor(data[DMX_MOTOR_A], PIN_MOTOR_A_1, PIN_MOTOR_A_2, PWM_CHANNEL_A, 0);
            controlMotor(data[DMX_MOTOR_B], PIN_MOTOR_B_1, PIN_MOTOR_B_2, PWM_CHANNEL_B, 1);
            digitalWrite(PIN_RELAY_1, data[DMX_RELAY_1] > 127);
            digitalWrite(PIN_RELAY_2, data[DMX_RELAY_2] > 127);
            controlServo(servo1, data[DMX_SERVO_1], lastServoValues[0], Servo1MinMicros, Servo1MaxMicros, SERVO_1_REVERSED);
            controlServo(servo2, data[DMX_SERVO_2], lastServoValues[1], Servo2MinMicros, Servo2MaxMicros, SERVO_2_REVERSED);
            controlServo(servo3, data[DMX_SERVO_3], lastServoValues[2], Servo3MinMicros, Servo3MaxMicros, SERVO_3_REVERSED);
            controlServo(servo4, data[DMX_SERVO_4], lastServoValues[3], Servo4MinMicros, Servo4MaxMicros, SERVO_4_REVERSED);
            controlStepper();
        }
        else
        {
            dmxIsConnected = false;
        }
    }
    else
    {
        dmxIsConnected = false;
    }*/

    if (homingActive) {
        uint16_t sg_result = tmc2209.SG_RESULT();
        Serial.printf("Homing SG %d\n", sg_result);
        // Check for stall detection
        /*if (tmc2209.SG_RESULT() < DEFAULT_STEPPER_STALL_THRESHOLD) {  
            Serial.println("✅ Homing Complete!");
            stepper.setCurrentPosition(0);
            stepper.stop();
            homingActive = false;
        }*/

        // Check for timeout or excessive steps
        if ((millis() - homingStartTime > DEFAULT_STEPPER_HOMING_TIMEOUT_MS) || abs(stepper->getCurrentPosition()) >= DEFAULT_STEPPER_HOMING_STEP_LIMIT) {
            Serial.println("❌ Homing Failed: Timeout or Step Limit Exceeded!");
            stepper->stopMove();
            homingActive = false;
            homingFailed = true;
        }
    }  

    // Handle button press logic
    // 🛠️ Debounce logic
    bool rawButtonState = digitalRead(PIN_BUTTON);
    if (rawButtonState != buttonPressed) {
        if (millis() - lastButtonPressTime > BUTTON_DEBOUNCE_DELAY_MS) {
            buttonPressed = rawButtonState;
            Serial.printf("Button State Updated: %s\n", buttonPressed ? "PRESSED" : "RELEASED");
            if (buttonPressed)
            {
                webConfig.startWiFi();
            }
        }
        lastButtonPressTime = millis();
    }


    webConfig.handleClient();

    oledDisplay.updateDMXInfo(baseDMX, DMX_LAST, dmxIsConnected, webConfig.isWiFiActive(), macSuffix);
}
