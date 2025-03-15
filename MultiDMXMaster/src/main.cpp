#include <WiFi.h>
#include <WebServer.h>
#include <esp_dmx.h>

uint8_t mac[6];

const char* ssid = "ESP32-DMX";

#define PIN_RS485_TX   17
#define PIN_RS485_RX   16
#define PIN_RS485_EN   21 

WebServer server(80);

byte data[DMX_PACKET_SIZE] = {0};
bool dmxEnabled = false;  // DMX output toggle
bool dmxDriverEnabled = false;  // DMX output toggle

dmx_port_t dmxPort = 1;

String createHTML() {
    String html = "<html><head><title>DMX Controller</title>";
    html += "<style>body{font-family:Arial;text-align:center;}input{width:80%;}</style>";
    html += "</head><body><h2>ESP32 DMX Controller</h2>";

    // Toggle button for enabling/disabling DMX
    html += "<button onclick='toggleDMX()'>";
    html += dmxEnabled ? "Disable DMX" : "Enable DMX";
    html += "</button><br><br>";

    // DMX sliders
    for (int i = 0; i < 10; i++) {
        html += "<p>Channel " + String(i + 1) + ": ";
        html += "<input type='range' min='0' max='255' value='" + String(data[i+1]) + "' ";
        html += "oninput='updateDMX(" + String(i) + ",this.value)'></p>";
    }

    // JavaScript for sending DMX updates and toggling
    html += "<script>function updateDMX(ch, val) {";
    html += "fetch('/dmx?ch=' + ch + '&val=' + val);}";

    html += "function toggleDMX() {";
    html += "fetch('/toggleDMX').then(() => location.reload());}</script>";

    html += "</body></html>";
    return html;
}

void handleRoot() {
  server.send(200, "text/html", createHTML());
}

void handleDMX() {
  if (server.hasArg("ch") && server.hasArg("val")) {
      int ch = server.arg("ch").toInt();
      int val = server.arg("val").toInt();
      if (ch >= 0 && ch < 10) {
          data[ch+1] = val;
          if(dmxEnabled)
          {
            Serial.printf("Channel %d -> %d\n", ch + 1, val);
          }
      }
  }
  server.send(200, "text/plain", "OK");
}

void handleToggleDMX() {
  dmxEnabled = !dmxEnabled;
  Serial.println(dmxEnabled ? "DMX Enabled" : "DMX Disabled");
  server.send(200, "text/plain", "OK");
}

void setup() {
    Serial.begin(115200);

    uint8_t mac[6];
    char ssid[20];
    WiFi.macAddress(mac);
    snprintf(ssid, sizeof(ssid), "DMXMaster-%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);

    // Setup WiFi in AP mode
    WiFi.softAP(ssid);
    Serial.println("WiFi AP Started");

    dmx_config_t config = DMX_CONFIG_DEFAULT;
    dmx_personality_t personalities[] = {};
    int personality_count = 0;
    dmx_driver_install(dmxPort, &config, personalities, personality_count);
    dmx_set_pin(dmxPort, PIN_RS485_TX, PIN_RS485_RX, PIN_RS485_EN);

    // Serve the web interface
    server.on("/", handleRoot);
    server.on("/dmx", handleDMX);
    server.on("/toggleDMX", handleToggleDMX);

    server.begin();
}

void loop() {
    server.handleClient();
    if (dmxEnabled) {
      if(!dmxDriverEnabled)
      {
        Serial.println("DMX enable");
        dmx_driver_enable(dmxPort);
        dmxDriverEnabled = true;
      }

      dmx_write(dmxPort, data, DMX_PACKET_SIZE);

      /* Now we can transmit the DMX packet! */
      dmx_send_num(dmxPort, DMX_PACKET_SIZE);

      /* We can do some other work here if we want. */

      /* If we have no more work to do, we will wait until we are done sending our
        DMX packet. */
      dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);
    }
    else
    {
      if(dmxDriverEnabled)
      {
        Serial.println("DMX disable");
        dmx_driver_disable(dmxPort);
        dmxDriverEnabled = false;
      }
    }
}
