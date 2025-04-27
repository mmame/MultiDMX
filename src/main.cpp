#include <Arduino.h>
#include <esp_dmx.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include "display.h"
#include <Wire.h>
#include "webpage.h"
#include "SoftwareSerial.h"
#include "dmxmap.h"

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

#define BUTTON_DEBOUNCE_DELAY_MS 100

// Stepper direction (true = reversed, false = normal)
const bool STEPPER_REVERSED = false;

Display oledDisplay(PIN_I2C_SDA, PIN_I2C_SCL);

// ✅ Global Variables
volatile int32_t stepperPosition = 0;  // Hardware-tracked step count
dmx_port_t dmxPort = DMX_NUM_1;
bool dmxIsConnected = false;
unsigned long lastUpdate = millis();
volatile bool buttonPressed = false;  // Updated globally in readDIPSwitch()
unsigned long lastButtonPressTime = 0;  // Track last state change

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
WebConfig webConfig;

TaskHandle_t fastTaskHandle = NULL;

// Fast Task Function (Runs on Core 1)
void FastTask(void *pvParameters) {
    while(true)
    {
        if (homingActive) {
            uint16_t sg_result = tmc2209.SG_RESULT();
            Serial.printf("Homing SG %d\n", sg_result);

            if (sg_result < webConfig.getStepperStallThreshold()) {  
                Serial.println("✅ Homing Complete!");
                stepper->setCurrentPosition(0);
                stepper->forceStop();
                homingActive = false;
            } 
            else if ((millis() - homingStartTime > webConfig.getStepperHomingTimeout()) ||
                    abs(stepper->getCurrentPosition()) >= webConfig.getStepperHomingStepLimit()) {
                Serial.println("❌ Homing Failed: Timeout or Step Limit Exceeded!");
                stepper->setCurrentPosition(0);
                stepper->forceStop();
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

        vTaskDelay(100 / portTICK_PERIOD_MS);  // Allow task switching
    }
}

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
        stepper->setSpeedInHz(webConfig.getStepperHomingSpeed());  
        stepper->setAcceleration(webConfig.getStepperHomingAccel());  
        stepper->moveTo(webConfig.isStepperReversed() ? webConfig.getStepperHomingStepLimit() : -webConfig.getStepperHomingStepLimit());  // Move indefinitely in reverse
    } 
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setup...");
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
    webConfig.baseDMX = readDIPSwitch();
    Serial.printf("Base DMX Address: %d\n", webConfig.baseDMX);

    // --- Configure TMC2209 ---
    pinMode(PIN_TMC2209_STEP, OUTPUT);
    pinMode(PIN_TMC2209_DIR, OUTPUT);

    //tmc2209Serial.begin(9600, SERIAL_8N1, PIN_TMC2209_UART, PIN_TMC2209_UART);
    tmc2209.begin();
    delay(100);
    Serial.println("\n=== TMC2209 UART Communication Check ===");

    Serial.printf(
        "IFCNT: %d\n",
        tmc2209.IFCNT()
    );

    Serial.println("========================================");

    //Stepping
    tmc2209.mres(16);

    //No analog VREF
    tmc2209.I_scale_analog(false);

    // Enable internal sense resistors
    tmc2209.internal_Rsense(true);

    tmc2209.rms_current(webConfig.getStepperCurrent());
    tmc2209.toff(0);
    tmc2209.pwm_autoscale(true);

    // Configure sensorless homing (StallGuard)
    tmc2209.TCOOLTHRS(0xFFFFF);
    //tmc2209.SGTHRS(webConfig.getStepperStallThreshold());
    tmc2209.iholddelay(10);

    engine.init();    
    stepper = engine.stepperConnectToPin(PIN_TMC2209_STEP);
    stepper->setDirectionPin(PIN_TMC2209_DIR);
    stepper->setSpeedInHz(webConfig.getStepperMaxSpeed());
    stepper->setAcceleration(webConfig.getStepperAccel());    
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

    tmc2209.toff(3);

    // Create Homing Task on Core 1
    xTaskCreatePinnedToCore(
        FastTask,  // Task function
        "FastTask",  // Task name
        4096,  // Stack size
        NULL,  // Task parameters
        1,  // Priority
        &fastTaskHandle,  // Task handle
        1  // Run on Core 1
    );
        
    stepperStartHoming();

    Serial.println("TMC2209 READY.");
}

void controlStepper(const uint8_t *dmxData) {
    int dmxSpeed = dmxData[DMX_STEPPER_SPEED(webConfig.baseDMX)];  
    int dmxTarget = dmxData[DMX_STEPPER_POSITION(webConfig.baseDMX)];

    webConfig.dmxRaw[6] = dmxData[DMX_STEPPER_SPEED(webConfig.baseDMX)];
    webConfig.dmxRaw[7] = dmxData[DMX_STEPPER_POSITION(webConfig.baseDMX)];

    long stepperMaxSpeed = map(dmxSpeed, 0, 255, 1, webConfig.getStepperMaxSpeed());  
    stepper->setSpeedInHz(stepperMaxSpeed);

    int newTarget = dmxTarget * webConfig.getStepperScale();

    if (STEPPER_REVERSED) {
        newTarget = -newTarget;  // Reverse direction
    }

    if (255 == dmxData[DMX_STEPPER_POSITION(webConfig.baseDMX)] && !homingActive) {  
        stepperStartHoming();
    } 
    else if (!homingActive && newTarget != stepperTargetPosition) {  
        stepperTargetPosition = newTarget;
        stepper->moveTo(webConfig.isStepperReversed() ? -stepperTargetPosition : stepperTargetPosition);
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

        webConfig.dmxRaw[4 + motorIndex] = dmxValue;
        webConfig.deviceState[4 + motorIndex] = String(speed) + " (" + (direction ? "Rev" : "Fwd") + ")";
    }
}

void controlServo(Servo &servo, int dmxValue, int &lastDmxValue, int minMicros, int maxMicros, bool isReversed, int index)
{
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

    webConfig.dmxRaw[index] = dmxValue;
    webConfig.deviceState[index] = String(pulseWidth) + " µs";
}

void processOutputs(const uint8_t *dmxData) {
    static int lastServoValues[4] = {-1, -1, -1, -1};

    // Local fallback array
    uint8_t defaultData[512] = {0};
    
    // Motors
    controlMotor(dmxData[DMX_MOTOR_A(webConfig.baseDMX)], PIN_MOTOR_A_1, PIN_MOTOR_A_2, PWM_CHANNEL_A, 0);
    controlMotor(dmxData[DMX_MOTOR_B(webConfig.baseDMX)], PIN_MOTOR_B_1, PIN_MOTOR_B_2, PWM_CHANNEL_B, 1);

    // Relays
    digitalWrite(PIN_RELAY_1, dmxData[DMX_RELAY_1(webConfig.baseDMX)] > 127);
    digitalWrite(PIN_RELAY_2, dmxData[DMX_RELAY_2(webConfig.baseDMX)] > 127);
    webConfig.dmxRaw[8] = dmxData[DMX_RELAY_1(webConfig.baseDMX)];
    webConfig.deviceState[8] = dmxData[DMX_RELAY_1(webConfig.baseDMX)] > 127 ? "ON" : "OFF";
    webConfig.dmxRaw[9] = dmxData[DMX_RELAY_2(webConfig.baseDMX)];
    webConfig.deviceState[9] = dmxData[DMX_RELAY_2(webConfig.baseDMX)] > 127 ? "ON" : "OFF";

    // Servos
    controlServo(servo1, dmxData[DMX_SERVO_1(webConfig.baseDMX)], lastServoValues[0],
                 webConfig.getServoMinMicros(1), webConfig.getServoMaxMicros(1),
                 webConfig.isServoReversed(1), 0);
    controlServo(servo2, dmxData[DMX_SERVO_2(webConfig.baseDMX)], lastServoValues[1],
                 webConfig.getServoMinMicros(2), webConfig.getServoMaxMicros(2),
                 webConfig.isServoReversed(2), 1);
    controlServo(servo3, dmxData[DMX_SERVO_3(webConfig.baseDMX)], lastServoValues[2],
                 webConfig.getServoMinMicros(3), webConfig.getServoMaxMicros(3),
                 webConfig.isServoReversed(3), 2);
    controlServo(servo4, dmxData[DMX_SERVO_4(webConfig.baseDMX)], lastServoValues[3],
                 webConfig.getServoMinMicros(4), webConfig.getServoMaxMicros(4),
                 webConfig.isServoReversed(4), 3);

    // Stepper
    controlStepper(dmxData);
}

void loop() {
    byte data[DMX_PACKET_SIZE];
    dmx_packet_t packet;

    webConfig.baseDMX = readDIPSwitch();

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
        if (!packet.err) {
            dmx_read(dmxPort, data, packet.size);
            if(data[0] == 0x00)
            {
                dmxIsConnected = true;
            }
            else
            {
                dmxIsConnected = false;
            }
        }
        else
        {
            dmxIsConnected = false;
        }
    }
    else
    {
        dmxIsConnected = false;
    }

    processOutputs(dmxIsConnected ? data : webConfig.getDefaultDMXValues());

    webConfig.handleClient();

    oledDisplay.updateDMXInfo(webConfig.baseDMX, DMX_LAST(webConfig.baseDMX), dmxIsConnected, webConfig.isWiFiActive(), webConfig.macSuffix);
}
