// webpage.h
#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

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
    int getStepperHomingAccel();
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
    void handleConfig();
};

#endif