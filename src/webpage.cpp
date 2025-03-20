// webpage.cpp
#include "webpage.h"

#define DEFAULT_STEPPER_TMC2209_CURRENT 1000
#define DEFAULT_STEPPER_SCALE 10000
#define DEFAULT_STEPPER_STALL_THRESHOLD 10
#define DEFAULT_STEPPER_SPEED 40000
#define DEFAULT_STEPPER_ACCEL 60000
#define DEFAULT_STEPPER_HOMING_SPEED 40000
#define DEFAULT_STEPPER_HOMING_ACCEL 60000
#define DEFAULT_STEPPER_HOMING_TIMEOUT_MS 10000
#define DEFAULT_STEPPER_HOMING_STEP_LIMIT 1000000
#define DEFAULT_SERVO_MICROS_MIN 500
#define DEFAULT_SERVO_MICROS_MAX 2500

#define WIFI_INITIAL_ACTIVE_DURATION_MS 60000  // 60 seconds after power-up
#define WIFI_ACTIVE_DURATION_MS 300000  // 5 minutes



const char webpageHTML[] PROGMEM = R"rawliteral(
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>MultiDMX Config</title>
        <script>
            document.addEventListener("DOMContentLoaded", function() {
                fetch("/config")
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById("stepper_current").value = data.stepper_current;
                        document.getElementById("stepper_scale").value = data.stepper_scale;
                        document.getElementById("stepper_max_speed").value = data.stepper_max_speed;
                        document.getElementById("stepper_accel").value = data.stepper_accel;
                        document.getElementById("stepper_stall").value = data.stepper_stall;
                        document.getElementById("stepper_homing_speed").value = data.stepper_homing_speed;
                        document.getElementById("stepper_homing_accel").value = data.stepper_homing_accel;
                        document.getElementById("stepper_homing_timeout").value = data.stepper_homing_timeout;
                        document.getElementById("stepper_homing_step_limit").value = data.stepper_homing_step_limit;
                        document.getElementById("stepper_reversed").checked = data.stepper_reversed;
                        
                        for (let i = 1; i <= 4; i++) {
                            document.getElementById("servo" + i + "_min").value = data["servo" + i + "_min"];
                            document.getElementById("servo" + i + "_max").value = data["servo" + i + "_max"];
                            document.getElementById("servo" + i + "_reversed").checked = data["servo" + i + "_reversed"];
                        }
                    });
            });
        </script>
    </head>
    <body>
        <h2>MultiDMX Configuration</h2>
        <p>For more information, see <a href="https://github.com/mmame/MultiDMX">https://github.com/mmame/MultiDMX</a></p>
        <form action='/save' method='POST'>
            Stepper Current: <input type='number' id='stepper_current' name='stepper_current'><br>
            Stepper Scaling Factor: <input type='number' id='stepper_scale' name='stepper_scale'><br>
            Stepper Max Speed (steps/s): <input type='number' id='stepper_max_speed' name='stepper_max_speed'><br>
            Stepper Acceleration (steps/s²): <input type='number' id='stepper_accel' name='stepper_accel'><br>
            Stepper Homing Stall Threshold: <input type='number' id='stepper_stall' name='stepper_stall'><br>
            Stepper Homing Speed (steps/s): <input type='number' id='stepper_homing_speed' name='stepper_homing_speed'><br>
            Stepper Homing Acceleration (steps/s²): <input type='number' id='stepper_homing_accel' name='stepper_homing_accel'><br>
            Stepper Homing Timeout (ms): <input type='number' id='stepper_homing_timeout' name='stepper_homing_timeout'><br>
            Stepper Homing Step Limit: <input type='number' id='stepper_homing_step_limit' name='stepper_homing_step_limit'><br>
            Stepper Reversed: <input type='checkbox' id='stepper_reversed' name='stepper_reversed'><br><br>
    
            <script>
                for (let i = 1; i <= 4; i++) {
                    document.write("Servo " + i + " Min Micros: <input type='number' id='servo" + i + "_min' name='servo" + i + "_min'><br>");
                    document.write("Servo " + i + " Max Micros: <input type='number' id='servo" + i + "_max' name='servo" + i + "_max'><br>");
                    document.write("Servo " + i + " Reversed: <input type='checkbox' id='servo" + i + "_reversed' name='servo" + i + "_reversed'><br><br>");
                }
            </script>
    
            <br><input type='submit' value='Save Settings'>
        </form><br>
    
        <form action='/reset' method='POST'>
            <input type='submit' value='Reset to Defaults' style='background-color: red; color: white;'>
        </form>
    </body>
    </html>
    )rawliteral";
    

WebConfig::WebConfig() : server(80) {}

void WebConfig::begin() {
    preferences.begin("config", false);  // Open preferences in read/write mode
    server.on("/", std::bind(&WebConfig::handleRoot, this));
    server.on("/save", std::bind(&WebConfig::handleSave, this));
    server.on("/reset", std::bind(&WebConfig::handleReset, this));
    server.on("/config", std::bind(&WebConfig::handleConfig, this));

    // Generate MAC-based SSID suffix
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(macSuffix, sizeof(macSuffix), "%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);

    // Start WiFi at boot and set stop time 60 seconds later
    startWiFi();
    wifiStopTime = millis() + WIFI_INITIAL_ACTIVE_DURATION_MS;
}

void WebConfig::handleConfig() {
    String json = "{";
    json += "\"stepper_current\":" + String(getStepperCurrent()) + ",";
    json += "\"stepper_scale\":" + String(getStepperScale()) + ",";
    json += "\"stepper_max_speed\":" + String(getStepperMaxSpeed()) + ",";
    json += "\"stepper_accel\":" + String(getStepperAccel()) + ",";
    json += "\"stepper_stall\":" + String(getStepperStallThreshold()) + ",";
    json += "\"stepper_homing_speed\":" + String(getStepperHomingSpeed()) + ",";
    json += "\"stepper_homing_accel\":" + String(getStepperHomingAccel()) + ",";
    json += "\"stepper_homing_timeout\":" + String(getStepperHomingTimeout()) + ",";
    json += "\"stepper_homing_step_limit\":" + String(getStepperHomingStepLimit()) + ",";
    json += "\"stepper_reversed\":" + String(isStepperReversed() ? "true" : "false") + ",";

    for (int i = 1; i <= 4; i++) {
        json += "\"servo" + String(i) + "_min\":" + String(getServoMinMicros(i)) + ",";
        json += "\"servo" + String(i) + "_max\":" + String(getServoMaxMicros(i)) + ",";
        json += "\"servo" + String(i) + "_reversed\":" + String(isServoReversed(i) ? "true" : "false");
        if (i < 4) json += ",";
    }
    json += "}";

    server.send(200, "application/json", json);
}

void WebConfig::handleReset() {
    preferences.clear();  // Erase all stored settings
    server.send(200, "text/plain", "Settings reset to defaults. Restarting...");
    Serial.println("Resetting to default settings, restarting ESP32...");
    delay(1000);
    ESP.restart();
}

void WebConfig::startWiFi() {
    if (!wifiActive) {
        char ssid[16];
        snprintf(ssid, sizeof(ssid), "MultiDMX-%s", macSuffix);
        Serial.printf("Starting WiFi AP: %s\n", ssid);
        WiFi.softAP(ssid);
        Serial.println("WiFi AP started. Access at 192.168.4.1");
        server.begin();
        wifiActive = true;
    }
    // Set the initial WiFi stop time (if called manually, extend by normal duration)
    wifiStopTime = millis() + WIFI_ACTIVE_DURATION_MS;
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
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.send(200, "text/html", webpageHTML);
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
    if (server.hasArg("stepper_accel")) {
        preferences.putInt("stepper_accel", server.arg("stepper_accel").toInt());
        configChanged = true;
    }
    if (server.hasArg("stepper_homing_accel")) {
        preferences.putInt("stepper_homing_accel", server.arg("stepper_homing_accel").toInt());
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
    if (server.hasArg("stepper_homing_speed")) {
        preferences.putInt("stepper_homing_speed", server.arg("stepper_homing_speed").toInt());
        configChanged = true;
    }
    if (server.hasArg("stepper_homing_timeout")) {
        preferences.putInt("stepper_homing_timeout", server.arg("stepper_homing_timeout").toInt());
        configChanged = true;
    }
    if (server.hasArg("stepper_homing_step_limit")) {
        preferences.putInt("stepper_homing_step_limit", server.arg("stepper_homing_step_limit").toInt());
        configChanged = true;
    }
    if (server.hasArg("stepper_reversed")) {
        preferences.putBool("stepper_reversed", server.arg("stepper_reversed") == "on");
        configChanged = true;
    }

    for (int i = 1; i <= 4; i++) {
        String minKey = "servo" + String(i) + "_min";
        String maxKey = "servo" + String(i) + "_max";
        String revKey = "servo" + String(i) + "_reversed";

        if (server.hasArg(minKey)) {
            preferences.putInt(minKey.c_str(), server.arg(minKey).toInt());
            configChanged = true;
        }
        if (server.hasArg(maxKey)) {
            preferences.putInt(maxKey.c_str(), server.arg(maxKey).toInt());
            configChanged = true;
        }
        if (server.hasArg(revKey)) {
            preferences.putBool(revKey.c_str(), server.arg(revKey) == "on");
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

int WebConfig::getStepperAccel() {
    return preferences.getInt("stepper_accel", DEFAULT_STEPPER_ACCEL);
}

int WebConfig::getStepperHomingAccel() {
    return preferences.getInt("stepper_homing_accel", DEFAULT_STEPPER_HOMING_ACCEL);
}

int WebConfig::getStepperStallThreshold() {
    return preferences.getInt("stepper_stall", DEFAULT_STEPPER_STALL_THRESHOLD);
}

int WebConfig::getStepperMaxSpeed() {
    return preferences.getInt("stepper_max_speed", DEFAULT_STEPPER_SPEED);
}

int WebConfig::getServoMinMicros(int servoIndex) {
    String key = "servo" + String(servoIndex) + "_min";
    return preferences.getInt(key.c_str(), DEFAULT_SERVO_MICROS_MIN);
}

int WebConfig::getServoMaxMicros(int servoIndex) {
    String key = "servo" + String(servoIndex) + "_max";
    return preferences.getInt(key.c_str(), DEFAULT_SERVO_MICROS_MAX);
}

int WebConfig::getStepperHomingSpeed() {
    return preferences.getInt("stepper_homing_speed", DEFAULT_STEPPER_HOMING_SPEED);
}

int WebConfig::getStepperHomingTimeout() {
    return preferences.getInt("stepper_homing_timeout", DEFAULT_STEPPER_HOMING_TIMEOUT_MS);
}

int WebConfig::getStepperHomingStepLimit() {
    return preferences.getInt("stepper_homing_step_limit", DEFAULT_STEPPER_HOMING_STEP_LIMIT);
}

bool WebConfig::isStepperReversed() {
    return preferences.getBool("stepper_reversed", false); // Default: not reversed
}

bool WebConfig::isServoReversed(int servoIndex) {
    String key = "servo" + String(servoIndex) + "_reversed";
    return preferences.getBool(key.c_str(), false); // Default: not reversed
}
