// webpage.h
#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#define DEFAULT_SSID "yourSSID"
#define DEFAULT_PASSWORD "yourPASSWORD"
#define DEFAULT_STEPPER_TMC2209_CURRENT 800
#define DEFAULT_STEPPER_SCALE 100
#define DEFAULT_STEPPER_STALL_THRESHOLD 5
#define DEFAULT_STEPPER_MAX_SPEED 20000
#define DEFAULT_STEPPER_HOMING_SPEED 100000
#define DEFAULT_STEPPER_ACCEL 500
#define DEFAULT_STEPPER_HOMING_TIMEOUT_MS 10000
#define DEFAULT_STEPPER_HOMING_STEP_LIMIT 500000
#define DEFAULT_SERVO_MICROS_MIN 500
#define DEFAULT_SERVO_MICROS_MAX 2500

#define WIFI_INITIAL_ACTIVE_DURATION_MS 60000  // 60 seconds after power-up
#define WIFI_ACTIVE_DURATION_MS 300000  // 5 minutes

class WebConfig {
public:
    WebConfig();
    void begin();
    void handleClient();
    void startWiFi();
    void stopWiFi();
    bool isWiFiActive();

    int getStepperCurrent();
    int getStepperScale();
    int getStepperStallThreshold();
    int getStepperMaxSpeed();
    int getStepperHomingSpeed();
    int getStepperAccel();
    int getStepperHomingTimeout();
    int getStepperHomingStepLimit();
    int getServoMinMicros(int servoIndex);
    int getServoMaxMicros(int servoIndex);
    bool isStepperReversed();
    bool isServoReversed(int servoIndex);

private:
    WebServer server;
    Preferences preferences;
    unsigned long wifiStopTime;
    bool wifiActive;
    char macSuffix[5];

    void handleRoot();
    void handleSave();
    void handleReset();
};

#endif