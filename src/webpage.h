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
    char macSuffix[10];
    int baseDMX;
    uint8_t dmxRaw[10];
    String deviceState[10];    

private:
    WebServer server;
    Preferences preferences;
    unsigned long wifiStopTime;
    bool wifiActive;

    void touchWiFi();
    void handleRoot();
    void handleSave();
    void handleReset();
    void handleStatus();
    void handleConfig();
    void handleOTAUploadPage();
    void handleOTAUpload();    
};

#endif