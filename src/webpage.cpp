// webpage.cpp
#include "webpage.h"
#include <Update.h>
#include "dmxmap.h"

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

//Default values when no DMX signal is present
#define DEFAULT_SERVO1_DEFAULT_DMX_VALUE     0
#define DEFAULT_SERVO2_DEFAULT_DMX_VALUE     0
#define DEFAULT_SERVO3_DEFAULT_DMX_VALUE     0
#define DEFAULT_SERVO4_DEFAULT_DMX_VALUE     0
#define DEFAULT_MOTOR_A_DEFAULT_DMX_VALUE    0 
#define DEFAULT_MOTOR_B_DEFAULT_DMX_VALUE    0
#define DEFAULT_STEPPER_SPEED_DEFAULT_DMX_VALUE 0
#define DEFAULT_STEPPER_POSITION_DEFAULT_DMX_VALUE 0
#define DEFAULT_RELAY1_DEFAULT_DMX_VALUE     0    
#define DEFAULT_RELAY2_DEFAULT_DMX_VALUE     0

#define WIFI_INITIAL_ACTIVE_DURATION_MS 60000  // 60 seconds after power-up
#define WIFI_ACTIVE_DURATION_MS 300000  // 5 minutes



const char webpageHTML[] PROGMEM = R"rawliteral(
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            body {
                font-family: sans-serif;
                margin: 20px;
            }
            table {
                border-collapse: collapse;
                width: 100%;
                margin-top: 20px;
            }
            th, td {
                padding: 8px 12px;
                text-align: left;
                border: 1px solid #ddd;
            }
            input[type="number"], input[type="checkbox"] {
                margin: 5px;
            }
            h2, h3 {
                margin-top: 30px;
            }
        </style>
        <title>MultiDMX Config</title>
        <script>
            document.addEventListener("DOMContentLoaded", function() {
                fetch("/config")
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById("hwid").innerText = data.hwid;

                        // Update DMX Table
                        for (let i = 0; i <= 9; i++) {
                            document.getElementById("dmx" + i).innerText = data.baseDMX + i;
                        }

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

                        for (let i = 0; i <= 9; i++) {
                            document.getElementById("dmxdef" + i).value = data["dmxdef" + i];
                        }
                    });
            });
            setInterval(() => {
                fetch("/status")
                    .then(response => response.json())
                    .then(status => {
                        for (let i = 0; i <= 9; i++) {
                            document.getElementById("dmxval" + i).innerText = status["dmx" + i];
                            document.getElementById("state" + i).innerText = status["state" + i];
                        }
                    });
            }, 4000);
        </script>
    </head>
    <body>
        <h2>MultiDMX Configuration</h2>
        <p>Compiled on: )rawliteral" __DATE__ " " __TIME__ R"rawliteral(</p>
        <p><strong>Hardware ID:</strong> <span id="hwid">Loading...</span></p>
        <p>For more information, see <a href="https://github.com/mmame/MultiDMX">https://github.com/mmame/MultiDMX</a></p>
        <br/>
        <table border="1">
            <tr>
                <th>DMX Channel</th>
                <th>Function</th>
                <th>Value</th>
                <th>State</th>
            </tr>
            <tr>
                <td id="dmx0">Loading...</td>
                <td>Controls Servo 1 Angle, mapped 0-255 → 0-180°</td>
                <td id="dmxval0">...</td>
                <td id="state0">...</td>
            </tr>
            <tr>
                <td id="dmx1">Loading...</td>
                <td>Controls Servo 2 Angle, mapped 0-255 → 0-180°</td>
                <td id="dmxval1">...</td>
                <td id="state1">...</td>
            </tr>
            <tr>
                <td id="dmx2">Loading...</td>
                <td>Controls Servo 3 Angle, mapped 0-255 → 0-180°</td>
                <td id="dmxval2">...</td>
                <td id="state2">...</td>
            </tr>
            <tr>
                <td id="dmx3">Loading...</td>
                <td>Controls Servo 4 Angle, mapped 0-255 → 0-180°</td>
                <td id="dmxval3">...</td>
                <td id="state3">...</td>
            </tr>
            <tr>
                <td id="dmx4">Loading...</td>
                <td>Controls Motor A Speed & Direction, 1-127 reverse, 129-255 forward, 0/128: standstill</td>
                <td id="dmxval4">...</td>
                <td id="state4">...</td>
            </tr>
            <tr>
                <td id="dmx5">Loading...</td>
                <td>Controls Motor B Speed & Direction, 1-127 reverse, 129-255 forward, 0/128: standstill</td>
                <td id="dmxval5">...</td>
                <td id="state5">...</td>
            </tr>
            <tr>
                <td id="dmx6">Loading...</td>
                <td>Sets stepper speed, mapped 0-255 → 1-"Stepper Max Speed"</td>
                <td id="dmxval6">...</td>
                <td id="state6">...</td>
            </tr>
            <tr>
                <td id="dmx7">Loading...</td>
                <td>Sets stepper position, mapped 0-255 → 1-"Stepper Scaling Factor"</td>
                <td id="dmxval7">...</td>
                <td id="state7">...</td>
            </tr>
            <tr>
                <td id="dmx8">Loading...</td>
                <td>Controls Relay 1, 0-127 off, 128-255 on</td>
                <td id="dmxval8">...</td>
                <td id="state8">...</td>
            </tr>
            <tr>
                <td id="dmx9">Loading...</td>
                <td>Controls Relay 2, 0-127 off, 128-255 on</td>
                <td id="dmxval9">...</td>
                <td id="state9">...</td>
            </tr>
        </table>

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

            <h3>Default DMX Values (when no DMX signal)</h3>
            <table>
                <tr><td>Servo 1 Default:</td><td><input type='number' id='dmxdef0' name='dmxdef0' min='0' max='255'></td></tr>
                <tr><td>Servo 2 Default:</td><td><input type='number' id='dmxdef1' name='dmxdef1' min='0' max='255'></td></tr>
                <tr><td>Servo 3 Default:</td><td><input type='number' id='dmxdef2' name='dmxdef2' min='0' max='255'></td></tr>
                <tr><td>Servo 4 Default:</td><td><input type='number' id='dmxdef3' name='dmxdef3' min='0' max='255'></td></tr>
                <tr><td>Motor A Speed Default:</td><td><input type='number' id='dmxdef4' name='dmxdef4' min='0' max='255'></td></tr>
                <tr><td>Motor B Speed Default:</td><td><input type='number' id='dmxdef5' name='dmxdef5' min='0' max='255'></td></tr>
                <tr><td>Stepper Speed Default:</td><td><input type='number' id='dmxdef6' name='dmxdef6' min='0' max='255'></td></tr>
                <tr><td>Stepper Position Default:</td><td><input type='number' id='dmxdef7' name='dmxdef7' min='0' max='255'></td></tr>
                <tr><td>Relay 1 Default:</td><td><input type='number' id='dmxdef8' name='dmxdef8' min='0' max='255'></td></tr>
                <tr><td>Relay 2 Default:</td><td><input type='number' id='dmxdef9' name='dmxdef9' min='0' max='255'></td></tr>
            </table>

            <br><input type='submit' value='Save Settings'>
        </form><br>
    
        <form action='/reset' method='POST'>
            <input type='submit' value='Reset to Defaults' style='background-color: red; color: white;'>
        </form>
        <br><a href='/update'>OTA Firmware Update</a>        
    </body>
    </html>
    )rawliteral";
    

WebConfig::WebConfig() : server(80) {}

void WebConfig::begin() {
    for (int i = 0; i < 10; i++) {
        deviceState[i] = "N/A";
    }    
    preferences.begin("config", false);  // Open preferences in read/write mode
    server.on("/", std::bind(&WebConfig::handleRoot, this));
    server.on("/save", std::bind(&WebConfig::handleSave, this));
    server.on("/reset", std::bind(&WebConfig::handleReset, this));
    server.on("/config", std::bind(&WebConfig::handleConfig, this));
    server.on("/status", std::bind(&WebConfig::handleStatus, this));
    server.on("/update", HTTP_GET, [this]() { handleOTAUploadPage(); });
    server.on("/update", HTTP_POST, 
        [this]() { server.send(200, "text/plain", (Update.hasError()) ? "Update Failed" : "Update Success. Rebooting..."); },
        [this]() { handleOTAUpload(); }
    );
    
    // Generate MAC-based SSID suffix
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(macSuffix, sizeof(macSuffix), "%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);

    // Start WiFi at boot and set stop time 60 seconds later
    startWiFi();
    wifiStopTime = millis() + WIFI_INITIAL_ACTIVE_DURATION_MS;
}

void WebConfig::handleConfig() {
    touchWiFi();
    String json = "{";
    json += "\"hwid\":\"" + String(macSuffix) + "\",";
    json += "\"baseDMX\":" + String(baseDMX) + ","; 
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

    for (int i = 0; i <= 9; i++) {
        json += ",\"dmxdef" + String(i) + "\":" + String(getDefaultDMXValue(i));
    }
        
    json += "}";

    server.send(200, "application/json", json);
}

void WebConfig::handleStatus() {
    touchWiFi();
    String json = "{";
    for (int i = 0; i <= 9; i++) {
        json += "\"dmx" + String(i) + "\":" + String(dmxRaw[i]) + ",";
        json += "\"state" + String(i) + "\":\"" + deviceState[i] + "\"";
        if (i < 9) json += ",";
    }
    json += "}";
    server.send(200, "application/json", json);
}


void WebConfig::handleOTAUploadPage() {
    touchWiFi();
    String html = "<html><body><h2>OTA Firmware Update</h2>"
                  "<form method='POST' action='/update' enctype='multipart/form-data'>"
                  "<input type='file' name='firmware'><br><br>"
                  "<input type='submit' value='Upload & Update'>"
                  "</form></body></html>";
    server.send(200, "text/html", html);
}

void WebConfig::handleOTAUpload() {
    touchWiFi();
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Starting OTA update: %s\n", upload.filename.c_str());
        if (!Update.begin()) { // start with max available size
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.println("OTA update finished successfully");
            ESP.restart();
        } else {
            Update.printError(Serial);
        }
    }
}

void WebConfig::handleReset() {
    touchWiFi();
    preferences.clear();  // Erase all stored settings
    Serial.println("Resetting to default settings, restarting ESP32...");

    server.send(200, "text/html",
        "<html><head><meta charset='UTF-8'><title>Resetting...</title></head>"
        "<body style='font-family:sans-serif; text-align:center; margin-top:50px;'>"
        "<h2>🔄 Settings reset to defaults</h2>"
        "<p>The device will restart shortly...</p>"
        "<script>setTimeout(() => window.location.href = '/', 5000);</script>"
        "</body></html>"
    );

    delay(1500);  // Let the user see the message
    ESP.restart();
}


void WebConfig::startWiFi() {
    if (!wifiActive) {
        char ssid[16];
        snprintf(ssid, sizeof(ssid), "MDMX-%s", macSuffix);
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

void WebConfig::touchWiFi() {
    wifiStopTime = millis() + WIFI_ACTIVE_DURATION_MS;
    //Serial.println("WiFi timeout extended due to HTTP request");
}


void WebConfig::handleRoot() {
    touchWiFi();    
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.send(200, "text/html", webpageHTML);
}


void WebConfig::handleSave() {
    touchWiFi();    
    bool configChanged = false;

    struct StepperIntParam {
        const char* key;
        int (WebConfig::*getter)();  // Member function pointer
    };
    
    StepperIntParam stepperIntParams[] = {
        { "stepper_current", &WebConfig::getStepperCurrent },
        { "stepper_scale", &WebConfig::getStepperScale },
        { "stepper_accel", &WebConfig::getStepperAccel },
        { "stepper_homing_accel", &WebConfig::getStepperHomingAccel },
        { "stepper_stall", &WebConfig::getStepperStallThreshold },
        { "stepper_max_speed", &WebConfig::getStepperMaxSpeed },
        { "stepper_homing_speed", &WebConfig::getStepperHomingSpeed },
        { "stepper_homing_timeout", &WebConfig::getStepperHomingTimeout },
        { "stepper_homing_step_limit", &WebConfig::getStepperHomingStepLimit }
    };
    
    for (auto& param : stepperIntParams) {
        if (server.hasArg(param.key)) {
            int newVal = server.arg(param.key).toInt();
            int oldVal = (this->*param.getter)();  // Call member function on this
            if (newVal != oldVal) {
                preferences.putInt(param.key, newVal);
                configChanged = true;
            }
        }
    }
    
    // Stepper reversed
    if (server.hasArg("stepper_reversed")) {
        bool newVal = server.arg("stepper_reversed") == "on";
        if (newVal != isStepperReversed()) {
            preferences.putBool("stepper_reversed", newVal);
            configChanged = true;
        }
    }
    
    // Servo values
    for (int i = 1; i <= 4; i++) {
        String minKey = "servo" + String(i) + "_min";
        String maxKey = "servo" + String(i) + "_max";
        String revKey = "servo" + String(i) + "_reversed";
    
        if (server.hasArg(minKey)) {
            int newVal = server.arg(minKey).toInt();
            int oldVal = getServoMinMicros(i);
            if (newVal != oldVal) {
                preferences.putInt(minKey.c_str(), newVal);
                configChanged = true;
            }
        }
    
        if (server.hasArg(maxKey)) {
            int newVal = server.arg(maxKey).toInt();
            int oldVal = getServoMaxMicros(i);
            if (newVal != oldVal) {
                preferences.putInt(maxKey.c_str(), newVal);
                configChanged = true;
            }
        }
    
        if (server.hasArg(revKey)) {
            bool newVal = server.arg(revKey) == "on";
            bool oldVal = isServoReversed(i);
            if (newVal != oldVal) {
                preferences.putBool(revKey.c_str(), newVal);
                configChanged = true;
            }
        }
    }
   
    // DMX default values
    for (int i = 0; i <= 9; i++) {
        String key = "dmxdef" + String(i);
        if (server.hasArg(key)) {
            uint8_t newVal = server.arg(key).toInt();
            uint8_t oldVal = getDefaultDMXValue(i);
            if (newVal != oldVal) {
                preferences.putUChar(key.c_str(), newVal);
                configChanged = true;
            }
        }
    }

    if (configChanged) {
        server.send(200, "text/html",
            "<html><body><p><strong>Settings saved. Restarting...</strong></p>"
            "<script>setTimeout(() => window.location.href = '/', 5000);</script>"
            "</body></html>");
        Serial.println("Configuration changed, restarting ESP32...");
        delay(1000);
        ESP.restart();
    } else {
        server.send(200, "text/html",
            "<html><body><p>No changes detected. Returning to main screen...</p>"
            "<script>setTimeout(() => window.location.href = '/', 1000);</script>"
            "</body></html>");
        Serial.println("No configuration changes detected.");
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

uint8_t WebConfig::getDefaultDMXValue(int index) {
    if (index < 0 || index > 9) return 0; // Safety fallback

    String key = "dmxdef" + String(index);

    switch (index) {
        case 0: return preferences.getUChar(key.c_str(), DEFAULT_SERVO1_DEFAULT_DMX_VALUE);
        case 1: return preferences.getUChar(key.c_str(), DEFAULT_SERVO2_DEFAULT_DMX_VALUE);
        case 2: return preferences.getUChar(key.c_str(), DEFAULT_SERVO3_DEFAULT_DMX_VALUE);
        case 3: return preferences.getUChar(key.c_str(), DEFAULT_SERVO4_DEFAULT_DMX_VALUE);
        case 4: return preferences.getUChar(key.c_str(), DEFAULT_MOTOR_A_DEFAULT_DMX_VALUE);
        case 5: return preferences.getUChar(key.c_str(), DEFAULT_MOTOR_B_DEFAULT_DMX_VALUE);
        case 6: return preferences.getUChar(key.c_str(), DEFAULT_STEPPER_SPEED_DEFAULT_DMX_VALUE);
        case 7: return preferences.getUChar(key.c_str(), DEFAULT_STEPPER_POSITION_DEFAULT_DMX_VALUE);
        case 8: return preferences.getUChar(key.c_str(), DEFAULT_RELAY1_DEFAULT_DMX_VALUE);
        case 9: return preferences.getUChar(key.c_str(), DEFAULT_RELAY2_DEFAULT_DMX_VALUE);
        default: return 0;
    }
}

const uint8_t* WebConfig::getDefaultDMXValues() {
    static uint8_t defaultDMX[512] = {0};

    defaultDMX[DMX_SERVO_1(baseDMX)] = getDefaultDMXValue(0);
    defaultDMX[DMX_SERVO_2(baseDMX)] = getDefaultDMXValue(1);
    defaultDMX[DMX_SERVO_3(baseDMX)] = getDefaultDMXValue(2);
    defaultDMX[DMX_SERVO_4(baseDMX)] = getDefaultDMXValue(3);
    defaultDMX[DMX_MOTOR_A(baseDMX)] = getDefaultDMXValue(4);
    defaultDMX[DMX_MOTOR_B(baseDMX)] = getDefaultDMXValue(5);
    defaultDMX[DMX_STEPPER_SPEED(baseDMX)] = getDefaultDMXValue(6);
    defaultDMX[DMX_STEPPER_POSITION(baseDMX)] = getDefaultDMXValue(7);
    defaultDMX[DMX_RELAY_1(baseDMX)] = getDefaultDMXValue(8);
    defaultDMX[DMX_RELAY_2(baseDMX)] = getDefaultDMXValue(9);

    return defaultDMX;
}
