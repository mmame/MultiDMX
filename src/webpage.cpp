// webpage.cpp
#include "webpage.h"

WebConfig::WebConfig() : server(80) {}

void WebConfig::begin() {
    preferences.begin("config", false);  // Open preferences in read/write mode
    server.on("/", std::bind(&WebConfig::handleRoot, this));
    server.on("/save", std::bind(&WebConfig::handleSave, this));
    server.on("/reset", std::bind(&WebConfig::handleReset, this));

    // Generate MAC-based SSID suffix
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macSuffix[5];
    snprintf(macSuffix, sizeof(macSuffix), "%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);

    // Start WiFi at boot and set stop time 60 seconds later
    startWiFi(macSuffix);
    wifiStopTime = millis() + WIFI_INITIAL_ACTIVE_DURATION_MS;
}


void WebConfig::handleReset() {
    preferences.clear();  // Erase all stored settings
    server.send(200, "text/plain", "Settings reset to defaults. Restarting...");
    Serial.println("Resetting to default settings, restarting ESP32...");
    delay(1000);
    ESP.restart();
}

void WebConfig::startWiFi(const char* macSuffix) {
    if (!wifiActive) {
        char ssid[16];
        snprintf(ssid, sizeof(ssid), "MultiDMX-%s", macSuffix);
        Serial.printf("Starting WiFi AP: %s\n", ssid);
        WiFi.softAP(ssid, "password123");
        Serial.println("WiFi AP started. Access at 192.168.4.1");
        server.begin();
        wifiActive = true;

        // Set the initial WiFi stop time (if called manually, extend by normal duration)
        wifiStopTime = millis() + WIFI_ACTIVE_DURATION_MS;
    }
}

void WebConfig::stopWiFi() {
    if (wifiActive) {
        Serial.println("WiFi timeout reached. Disabling AP...");
        WiFi.softAPdisconnect(true);
        wifiActive = false;
    }
}

bool WebConfig::isWiFiActive() {
    return wifiActive;
}

void WebConfig::handleClient() {
    if (!wifiActive) return;  // Skip if WiFi is off

    server.handleClient();
    
    if (server.client()) {
        wifiStopTime = millis() + WIFI_ACTIVE_DURATION_MS;  // Extend timeout on interaction
        Serial.println("Web interaction detected, extending WiFi timeout.");
    }

    // Check if it's time to turn off WiFi
    if (millis() > wifiStopTime) {
        stopWiFi();
    }
}


void WebConfig::handleRoot() {
    String html = "<html><head><title>MultiDMX Config</title></head><body>"
                  "<h2>MultiDMX Configuration</h2>"
                  "<form action='/save' method='POST'>"
                  "Stepper Current: <input type='number' name='stepper_current' value='" + String(getStepperCurrent()) + "'><br>"
                  "Stepper Scale: <input type='number' name='stepper_scale' value='" + String(getStepperScale()) + "'><br>"
                  "Stepper Stall Threshold: <input type='number' name='stepper_stall' value='" + String(getStepperStallThreshold()) + "'><br>"
                  "Stepper Max Speed: <input type='number' name='stepper_max_speed' value='" + String(getStepperMaxSpeed()) + "'><br>";

    for (int i = 1; i <= 4; i++) {
        html += "Servo " + String(i) + " Min Micros: <input type='number' name='servo" + String(i) + "_min' value='" + String(getServoMinMicros(i)) + "'><br>";
        html += "Servo " + String(i) + " Max Micros: <input type='number' name='servo" + String(i) + "_max' value='" + String(getServoMaxMicros(i)) + "'><br>";
    }

    html += "<br><input type='submit' value='Save Settings'>"
            "</form><br>"
            "<form action='/reset' method='POST'>"
            "<input type='submit' value='Reset to Defaults' style='background-color: red; color: white;'>"
            "</form>"
            "</body></html>";

    server.send(200, "text/html", html);
}


void WebConfig::handleSave() {
    bool configChanged = false;

    if (server.hasArg("stepper_current")) {
        preferences.putInt("stepper_current", server.arg("stepper_current").toInt());
        configChanged = true;
    }
    if (server.hasArg("stepper_scale")) {
        preferences.putInt("stepper_scale", server.arg("stepper_scale").toInt());
        configChanged = true;
    }
    if (server.hasArg("stepper_stall")) {
        preferences.putInt("stepper_stall", server.arg("stepper_stall").toInt());
        configChanged = true;
    }
    if (server.hasArg("stepper_max_speed")) {
        preferences.putInt("stepper_max_speed", server.arg("stepper_max_speed").toInt());
        configChanged = true;
    }

    for (int i = 1; i <= 4; i++) {
        String minKey = "servo" + String(i) + "_min";
        String maxKey = "servo" + String(i) + "_max";

        if (server.hasArg(minKey)) {
            preferences.putInt(minKey.c_str(), server.arg(minKey).toInt());
            configChanged = true;
        }
        if (server.hasArg(maxKey)) {
            preferences.putInt(maxKey.c_str(), server.arg(maxKey).toInt());
            configChanged = true;
        }
    }

    server.send(200, "text/plain", "Settings saved. Restarting...");

    if (configChanged) {
        Serial.println("Configuration changed, restarting ESP32...");
        delay(1000);
        ESP.restart();
    }
}

int WebConfig::getStepperCurrent() {
    return preferences.getInt("stepper_current", DEFAULT_STEPPER_TMC2209_CURRENT);
}

int WebConfig::getStepperScale() {
    return preferences.getInt("stepper_scale", DEFAULT_STEPPER_SCALE);
}

int WebConfig::getStepperStallThreshold() {
    return preferences.getInt("stepper_stall", DEFAULT_STEPPER_STALL_THRESHOLD);
}

int WebConfig::getStepperMaxSpeed() {
    return preferences.getInt("stepper_max_speed", DEFAULT_STEPPER_MAX_SPEED);
}

int WebConfig::getServoMinMicros(int servoIndex) {
    String key = "servo" + String(servoIndex) + "_min";
    return preferences.getInt(key.c_str(), DEFAULT_SERVO_MICROS_MIN);
}

int WebConfig::getServoMaxMicros(int servoIndex) {
    String key = "servo" + String(servoIndex) + "_max";
    return preferences.getInt(key.c_str(), DEFAULT_SERVO_MICROS_MAX);
}
