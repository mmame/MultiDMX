#include <Arduino.h>
#include <esp_dmx.h>
#include <ESP32Servo.h>
#include <TMCStepper.h>

// ✅ DMX RS-485 Pins
#define PIN_RS485_TX   17
#define PIN_RS485_RX   16
#define PIN_RS485_EN   21  // Connected to both DE & RE

// ✅ DIP Switch (74HC165) Pins
//#define PIN_DIP_CLK    18  // Clock (SHCP)
//#define PIN_DIP_LATCH  5   // Latch (STCP)
#define PIN_DIP_DATA   34  // Serial Data Output (QH)

// ✅ Stepper Control Pins
#define PIN_TMC2209_UART  14  // TX & RX connected (Half-Duplex)
#define PIN_TMC2209_EN    15  // Enable (LOW to enable driver)
#define PIN_TMC2209_INDEX 32  // Step pulse counter input
#define TMC2209_CURRENT   600  // Motor current in mA
#define STEPPER_SCALE     100  // Position scaling factor
#define STALL_THRESHOLD   5    // Stall detection sensitivity (-64 to 63)

// ✅ Motor Control Pins (H-Bridge)
#define PIN_MOTOR_A_1  4
#define PIN_MOTOR_A_2  5
#define PIN_MOTOR_B_1  18
#define PIN_MOTOR_B_2  19

// ✅ Servo Control Pins
#define PIN_SERVO_1    23  // Servo 1 PWM pin
//#define PIN_SERVO_2    4  // Servo 2 PWM pin

// ✅ Relay Control Pins
#define PIN_RELAY_1    27
#define PIN_RELAY_2    26

// ✅ Centralized DMX Base Addresses
#define DMX_MOTOR       (baseDMX + 1)
#define DMX_SERVO_1     (baseDMX + 2)
#define DMX_SERVO_2     (baseDMX + 3)
#define DMX_SPEED       (baseDMX + 4)
#define DMX_POSITION    (baseDMX + 5)
#define DMX_RELAY_1     (baseDMX + 6)
#define DMX_RELAY_2     (baseDMX + 7)

// ✅ Global Variables
volatile int32_t stepperPosition = 0;  // Hardware-tracked step count
dmx_port_t dmxPort = 1;
byte data[DMX_PACKET_SIZE];
bool dmxIsConnected = false;
unsigned long lastUpdate = millis();
bool homingActive = false;  // Flag to track homing state
uint16_t baseDMX = 1;  // Base DMX Address

// Servo objects
Servo servo1, servo2;

// Stepper Driver
HardwareSerial stepperSerial(2);
TMC2209Stepper stepper(&stepperSerial, 0.11f, PIN_TMC2209_EN);

// Interrupt Service Routine (ISR) for counting steps
void IRAM_ATTR countSteps() {
    if (stepper.VACTUAL() > 0) {
        stepperPosition++;
    } else {
        stepperPosition--;
    }
}

// ✅ Read DIP Switch (74HC165) to Determine Base DMX Address
uint16_t readDIPSwitch() {
    uint16_t value = 0;
    /*digitalWrite(PIN_DIP_LATCH, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_DIP_LATCH, HIGH);

    for (int i = 0; i < 16; i++) {
        value |= (digitalRead(PIN_DIP_DATA) << i);
        digitalWrite(PIN_DIP_CLK, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_DIP_CLK, LOW);
    }*/

    return value & 0x01FF;  // Extract lower 9 bits (DMX Address)
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setup...");

    // --- Configure DIP Switch Pins ---
    //pinMode(PIN_DIP_CLK, OUTPUT);
    //pinMode(PIN_DIP_LATCH, OUTPUT);
    pinMode(PIN_DIP_DATA, INPUT);
    //digitalWrite(PIN_DIP_CLK, LOW);
    //digitalWrite(PIN_DIP_LATCH, HIGH);


    pinMode(PIN_MOTOR_A_1, OUTPUT);
    pinMode(PIN_MOTOR_A_2, OUTPUT);
    pinMode(PIN_MOTOR_B_1, OUTPUT);
    pinMode(PIN_MOTOR_B_2, OUTPUT);
    digitalWrite(PIN_MOTOR_A_1, LOW);
    digitalWrite(PIN_MOTOR_A_2, LOW);
    digitalWrite(PIN_MOTOR_B_1, LOW);
    digitalWrite(PIN_MOTOR_B_2, LOW);


    // --- Read DIP Switch ---
    baseDMX = readDIPSwitch();
    Serial.printf("Base DMX Address: %d\n", baseDMX);

    // --- Configure TMC2209 ---
    pinMode(PIN_TMC2209_EN, OUTPUT);
    digitalWrite(PIN_TMC2209_EN, LOW);
    stepperSerial.begin(115200, SERIAL_8N1, PIN_TMC2209_UART, PIN_TMC2209_UART);
    stepper.begin();
    stepper.rms_current(TMC2209_CURRENT);
    stepper.toff(4);
    stepper.pwm_autoscale(true);

    // Enable INDEX_STEP mode to output step pulses
    stepper.GCONF(stepper.GCONF() | (1 << 10));

    // Configure sensorless homing (StallGuard)
    stepper.TCOOLTHRS(0xFFFFF);  // Enable StallGuard at low speeds
    stepper.SGTHRS(STALL_THRESHOLD);  // Set stall threshold

    // --- Setup ESP32 GPIO Interrupt for Step Counting ---
    pinMode(PIN_TMC2209_INDEX, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_TMC2209_INDEX), countSteps, RISING);

    // --- Configure DMX ---
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    dmx_driver_install(dmxPort, &config, NULL, 0);
    dmx_set_pin(dmxPort, PIN_RS485_TX, PIN_RS485_RX, PIN_RS485_EN);

    // --- Initialize Servos ---
    servo1.attach(PIN_SERVO_1);
    //servo2.attach(PIN_SERVO_2);
    servo1.write(90);
    //servo2.write(90);

    // --- Initialize Relays ---
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_RELAY_2, OUTPUT);
    digitalWrite(PIN_RELAY_1, LOW);
    digitalWrite(PIN_RELAY_2, LOW);

    Serial.println("TMC2209 READY.");
}

void controlStepper() {
    if (homingActive) return;

    int stepperSpeed = map(data[DMX_SPEED], 0, 255, 1, 500);
    int newTarget = data[DMX_POSITION] * STEPPER_SCALE;

    if (newTarget == 0) {
        Serial.println("Homing Stepper...");
        stepperPosition = 0;  
    } else if (newTarget != stepperPosition) {
        Serial.printf("Moving to position: %d steps\n", newTarget);
    }

    if (stepperPosition < newTarget) {
        stepper.shaft(true);
        stepper.VACTUAL(stepperSpeed);
    } else if (stepperPosition > newTarget) {
        stepper.shaft(false);
        stepper.VACTUAL(stepperSpeed);
    } else {
        stepper.VACTUAL(0);
    }

    Serial.printf("Stepper Position: %d | Speed: %d\n", stepperPosition, stepperSpeed);
}

void controlMotor() {
    int motorSpeed = map(data[DMX_MOTOR], 0, 255, -255, 255);
    
    if (motorSpeed > 0) {
        digitalWrite(PIN_MOTOR_A_1, HIGH);
        digitalWrite(PIN_MOTOR_A_2, LOW);
    } else {
        digitalWrite(PIN_MOTOR_A_1, LOW);
        digitalWrite(PIN_MOTOR_A_2, HIGH);
    }
    
    ledcWrite(0, abs(motorSpeed));  
}

void loop() {
    dmx_packet_t packet;

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
        if (!packet.err) {
            dmx_read(dmxPort, data, packet.size);
            //controlMotor();
            //digitalWrite(PIN_RELAY_1, data[DMX_RELAY_1] > 127);
            //digitalWrite(PIN_RELAY_2, data[DMX_RELAY_2] > 127);
            servo1.write(map(data[DMX_SERVO_1], 0, 255, 0, 180));
            servo2.write(map(data[DMX_SERVO_2], 0, 255, 0, 180));
            //controlStepper();
        }
    }
}
