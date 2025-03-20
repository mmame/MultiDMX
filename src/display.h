// display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define MAC_SUFFIX_LENGTH 8  // Last X bytes of the MAC for SSID in hex format

class Display {
public:
    Display(uint8_t sda, uint8_t scl);
    void begin();
    void updateDMXInfo(uint16_t baseAddress, uint16_t lastAddress, bool dmxStatus, bool wifiStatus, const char* macSuffix);

private:
    Adafruit_SSD1306 display;
    uint16_t lastBaseAddress;
    uint16_t lastDmxLastAddress;
    bool lastDmxStatus;
    bool lastWiFiStatus;
    char lastMacSuffix[MAC_SUFFIX_LENGTH + 1];
};

#endif
