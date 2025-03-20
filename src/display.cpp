// display.cpp
#include "display.h"
#include <WiFi.h>

Display::Display(uint8_t sda, uint8_t scl) : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET), lastBaseAddress(0), lastDmxLastAddress(0), lastDmxStatus(false), lastWiFiStatus(false) {
    memset(lastMacSuffix, 0, sizeof(lastMacSuffix));
    Wire.begin(sda, scl);
}

void Display::begin() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();
}

void Display::updateDMXInfo(uint16_t baseAddress, uint16_t lastAddress, bool dmxStatus, bool wifiStatus, const char* macSuffix) {
    if (baseAddress != lastBaseAddress || lastAddress != lastDmxLastAddress || dmxStatus != lastDmxStatus || wifiStatus != lastWiFiStatus || strcmp(macSuffix, lastMacSuffix) != 0) {
        lastBaseAddress = baseAddress;
        lastDmxLastAddress = lastAddress;
        lastDmxStatus = dmxStatus;
        lastWiFiStatus = wifiStatus;
        strncpy(lastMacSuffix, macSuffix, MAC_SUFFIX_LENGTH);
        lastMacSuffix[MAC_SUFFIX_LENGTH] = '\0';  // Ensure null termination

        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("DMX Addr: ");
        display.print(baseAddress);
        display.print("-");
        display.print(lastAddress);
        display.setCursor(0, 16);
        display.print("DMX: ");
        display.print(dmxStatus ? "Connected" : "Disconnected");
        display.setCursor(0, 32);
        display.print("WiFi: ");
        display.print(wifiStatus ? "192.168.4.1" : "OFF");
        display.setCursor(0, 48);
        display.print("SSID: MDMX-");
        display.print(macSuffix);
        display.display();
    }
}
