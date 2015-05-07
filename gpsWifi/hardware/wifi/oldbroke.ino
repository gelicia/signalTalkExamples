#include "wifiscan.h"

bool start_scan = true;

void setup() {
    Serial.begin(9600);
    WiFi.on();  
}



void loop() {
    static int lastScan = 0;    
    if ((millis()-lastScan)>30000 || start_scan ) {  // scan every 30s or if requested
        start_scan = true;
        lastScan = millis();
    }
    scan();   // start or continue scan
}
    
void scan() {
    static bool scanning = false;
    static WifiScan scanner;    
    if (!scanning && start_scan){
        scanning = true;
        start_scan = false;
        Serial.println("");
        scanner.startScan();
    }
    else {
        if (scanning){
            WifiScanResults_t result;
            scanning = scanner.next(result);
            String name = ssid(result);
            if (name != ""){
                Serial.println(ssid(result) + "," + result.security + "," + result.rssi);
            }
        }
    }
    
}


String ssid(WifiScanResults_t& result) {
    char buf[33];
    memcpy(buf, result.ssid, 32);
    buf[result.ssidlen] = 0;
    return buf;
}