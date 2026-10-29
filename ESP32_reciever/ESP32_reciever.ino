/*
 * SonoGuard Worker B Receiver (ESP32)
 * Acts as a peer safety node: listens for BLE hazard broadcasts from MAX32655
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define LED_PIN 2  // Standard onboard LED on most ESP32 DevKit boards

BLEScan* pBLEScan;
unsigned long alertResetTime = 0;
bool alertActive = false;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        // Check if the advertising device is SonoGuard
        if (advertisedDevice.haveName() && advertisedDevice.getName() == "SonoGuard") {
            
            String mfgData = advertisedDevice.getManufacturerData();
            
            Serial.println("\n========================================");
            Serial.println(" [!] PEER SAFETY RELAY ALERT DETECTED!");
            Serial.print(" Received from Peer Node: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
            
            if (mfgData.length() >= 4) {
                uint8_t hazardType = mfgData[0];
                uint16_t freq = (mfgData[1] << 8) | mfgData[2];
                uint8_t ratio = mfgData[3];

                Serial.print(" Hazard Type : ");
                Serial.println(hazardType == 1 ? "PULSED BEEPER" : "SWEEPING SIREN");
                Serial.printf(" Peak Frequency: %d Hz\n", freq);
                Serial.printf(" Tonal Ratio   : %d%%\n", ratio);
            }
            Serial.println(" ACTION: TRIGGERING WORKER B HEADSET ALERT");
            Serial.println("========================================\n");

            // Turn on Worker B's visual/audio alert indicator
            digitalWrite(LED_PIN, HIGH);
            alertActive = true;
            alertResetTime = millis() + 2000; // Keep alert on for 2 seconds
        }
    }
};

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("\n=== SonoGuard Peer Node (ESP32 Worker B) Booting ===");
    Serial.println("Role: BLE Safety Observer / Scanner");

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); // Active scan requests scan response
    pBLEScan->setInterval(100);    // Scan interval in ms
    pBLEScan->setWindow(99);       // Scan window in ms (near-continuous listening)

    Serial.println("Listening for peer hazard broadcasts...\n");
}

void loop() {
    // Scan in short 1-second non-blocking bursts
    pBLEScan->start(1, false);
    pBLEScan->clearResults();

    // Reset alert indicator after timeout
    if (alertActive && millis() > alertResetTime) {
        digitalWrite(LED_PIN, LOW);
        alertActive = false;
        Serial.println("[Worker B] Status returned to SAFE / MONITORING.");
    }
}