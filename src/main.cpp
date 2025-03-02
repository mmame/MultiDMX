#include <Arduino.h>
#include <esp_dmx.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

#include <Wire.h>

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
#define PIN_I2C_SCL    35  

#define PIN_BUTTON     36 

// ✅ Centralized DMX Base Addresses
#define DMX_SERVO_1             (baseDMX + 1)
#define DMX_SERVO_2             (baseDMX + 2)
#define DMX_SERVO_3             (baseDMX + 3)
#define DMX_SERVO_4             (baseDMX + 4)
#define DMX_MOTOR_A             (baseDMX + 5)
#define DMX_MOTOR_B             (baseDMX + 6)
#define DMX_STEPPER_SPEED       (baseDMX + 7)
#define DMX_STEPPER_POSITION    (baseDMX + 8)
#define DMX_RELAY_1             (baseDMX + 9)
#define DMX_RELAY_2             (baseDMX + 10)

#define TMC2209_CURRENT   600  // Motor current in mA
#define STEPPER_SCALE     100  // Position scaling factor
#define STALL_THRESHOLD   5    // Stall detection sensitivity (-64 to 63)
#define STEPPER_MAX_SPEED 2000 // Max speed in steps/sec
#define STEPPER_ACCEL     500  // Acceleration (steps/sec^2)

// ✅ Global Variables
volatile int32_t stepperPosition = 0;  // Hardware-tracked step count
dmx_port_t dmxPort = 1;
byte data[DMX_PACKET_SIZE];
bool dmxIsConnected = false;
unsigned long lastUpdate = millis();
uint16_t baseDMX = 1;  // Base DMX Address
bool buttonPressed = false;

#define DEFAULT_SERVO_MICROS_MIN  500
#define DEFAULT_SERVO_MICROS_MAX  2500

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
AccelStepper stepper(AccelStepper::DRIVER, PIN_TMC2209_STEP, PIN_TMC2209_DIR);
HardwareSerial tmc2209Serial(2);
TMC2209Stepper tmc2209(&tmc2209Serial, 0.11f, 0);
int32_t stepperTargetPosition = 0;  // Target position
bool homingActive = false; 

// Interrupt Service Routine (ISR) for counting steps
void IRAM_ATTR countSteps() {
    if (tmc2209.VACTUAL() > 0) {
        stepperPosition++;
    } else {
        stepperPosition--;
    }
}

// ✅ Read DIP Switch (74HC165) to Determine Base DMX Address
uint16_t readDIPSwitch() {
    uint16_t value = 0;
    digitalWrite(PIN_DIP_LATCH, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_DIP_LATCH, HIGH);

    for (int i = 0; i < 16; i++) {
        value |= (digitalRead(PIN_DIP_DATA) << i);
        digitalWrite(PIN_DIP_CLK, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_DIP_CLK, LOW);
    }

    return value & 0x01FF;  // Extract lower 9 bits (DMX Address)
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setup...");

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
    pinMode(PIN_BUTTON, INPUT_PULLUP); 

    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_A_1, PWM_CHANNEL_A);
    ledcAttachPin(PIN_MOTOR_B_1, PWM_CHANNEL_B);

    pinMode(PIN_TMC2209_STEP, OUTPUT);
    pinMode(PIN_TMC2209_DIR, OUTPUT);

    // --- Read DIP Switch ---
    baseDMX = readDIPSwitch();
    Serial.printf("Base DMX Address: %d\n", baseDMX);

    // --- Configure TMC2209 ---
    pinMode(PIN_TMC2209_STEP, OUTPUT);
    pinMode(PIN_TMC2209_DIR, OUTPUT);
    tmc2209Serial.begin(115200, SERIAL_8N1, PIN_TMC2209_UART, PIN_TMC2209_UART);
    tmc2209.begin();
    tmc2209.rms_current(TMC2209_CURRENT);
    tmc2209.toff(0);
    tmc2209.pwm_autoscale(true);

    // Configure sensorless homing (StallGuard)
    tmc2209.TCOOLTHRS(0xFFFFF);
    tmc2209.SGTHRS(STALL_THRESHOLD);

    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    stepper.setAcceleration(STEPPER_ACCEL);    
    stepper.setCurrentPosition(0);  // Assume starting at position 0

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

    Serial.println("TMC2209 READY.");
}

void controlStepper() {
    int dmxSpeed = data[DMX_STEPPER_SPEED];  
    int dmxTarget = data[DMX_STEPPER_POSITION];

    long stepperMaxSpeed = map(dmxSpeed, 0, 255, 100, 3000);  
    stepper.setMaxSpeed(stepperMaxSpeed);

    int newTarget = dmxTarget * STEPPER_SCALE;

    if (newTarget == 0 && !homingActive) {  
        Serial.println("🏠 Starting Homing...");
        homingActive = true;
        stepper.setMaxSpeed(1000);  
        stepper.setAcceleration(500);  
        stepper.moveTo(-50000);  // Move indefinitely in reverse
    } 
    else if (!homingActive && newTarget != stepperTargetPosition) {  
        stepperTargetPosition = newTarget;
        stepper.moveTo(stepperTargetPosition);
        Serial.printf("Moving to position: %d steps\n", stepperTargetPosition);
    }
}


void controlMotor(int dmxValue, int pin_pwm, int pin_direction, int pwm_channel) {
    int speed = 0;

    if (dmxValue == 0 || dmxValue == 128) {  // 🚦 Stop Motor
        digitalWrite(pin_direction, LOW);
        speed = 0;
    } else if (dmxValue < 128) {  // 🔄 Reverse
        speed = map(dmxValue, 1, 127, 1, 255);  // Reverse speed (1-255)
        digitalWrite(pin_direction, HIGH);
    } else {  // 🚀 Forward
        speed = map(dmxValue, 129, 255, 1, 255);  // Forward speed (1-255)
        digitalWrite(pin_direction, LOW);
    }
    ledcWrite(pwm_channel, speed);  // Set PWM speed

    Serial.printf("Motor on PWM %d: DMX=%d, Speed=%d\n", pwm_channel, dmxValue, speed);
}

void loop() {
    dmx_packet_t packet;

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
        if (!packet.err) {
            dmx_read(dmxPort, data, packet.size);
            controlMotor(data[DMX_MOTOR_A], PIN_MOTOR_A_1, PIN_MOTOR_A_2, PWM_CHANNEL_A);
            controlMotor(data[DMX_MOTOR_B], PIN_MOTOR_B_1, PIN_MOTOR_B_2, PWM_CHANNEL_B);
            digitalWrite(PIN_RELAY_1, data[DMX_RELAY_1] > 127);
            digitalWrite(PIN_RELAY_2, data[DMX_RELAY_2] > 127);
            servo1.writeMicroseconds(map(data[DMX_SERVO_1], 0, 255, Servo1MinMicros, Servo1MaxMicros));
            servo2.writeMicroseconds(map(data[DMX_SERVO_2], 0, 255, Servo2MinMicros, Servo2MaxMicros));
            servo3.writeMicroseconds(map(data[DMX_SERVO_3], 0, 255, Servo3MinMicros, Servo3MaxMicros));
            servo4.writeMicroseconds(map(data[DMX_SERVO_4], 0, 255, Servo4MinMicros, Servo4MaxMicros));
            controlStepper();
        }
    }

    if (digitalRead(PIN_BUTTON) == LOW && !buttonPressed) {
        Serial.println("Button Pressed!");
        buttonPressed = true;
    } else if (digitalRead(PIN_BUTTON) == HIGH && buttonPressed) {
        buttonPressed = false;
    }    

    if (homingActive) {
        stepper.run();  

        if (tmc2209.SG_RESULT() < STALL_THRESHOLD) {  
            Serial.println("✅ Homing Complete!");
            stepper.setCurrentPosition(0);
            stepper.stop();
            homingActive = false;
        }
    } else {
        stepper.run();  
    }    
}
