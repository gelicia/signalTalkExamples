#include "wifiscan.h"
#undef min
#undef max
#include <vector>
#include <algorithm>

std::vector<String> scanned;   // The raw list of SSIDs from this scan (includes already seen networks)
// This is used to know when the scan has "looped" around when the SSID has already been scanned previously

#include "Adafruit_GPS.h"
#include <math.h>

#include "sd-card-library/sd-card-library.h"

#include "neopixel/neopixel.h"

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

#define APP_VERSION 10

File myFile;
const uint8_t chipSelect = A2;
const uint8_t mosiPin = A5;
const uint8_t misoPin = A4;
const uint8_t clockPin = A3;

byte bufferSize = 64;
byte bufferIndex = 0;
char buffer[65];
char c;

bool start_scan = true;

String gpsLongG = "0";
String gpsLatG = "0";
uint8_t gpsFixG = 0;

#define PIXEL_PIN D5
#define PIXEL_COUNT 1
#define PIXEL_TYPE WS2812
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

void setup() {
    GPS.begin(9600);
    Serial.begin(9600);
    
    mySerial.begin(9600);
    
    pixel.begin();
    pixel.setPixelColor(0, pixel.Color(255,0,0));
    pixel.show(); 
    
    //while (!Serial.available());
    Serial.print("Initializing SD card...");
    // Initialize HARDWARE SPI with user defined chipSelect
    if (!SD.begin(chipSelect)) {
        Serial.println("initialization failed!");
        return;
    }
    Serial.print("SD Card Initialized");
    
    WiFi.on();  
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz
    
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_NOANTENNA);
    
    delay(1000);
    
    // Ask for firmware version
    mySerial.println(PMTK_Q_RELEASE);
    Spark.publish("GPS", "{ status: \"started up! "+String(APP_VERSION)+"\"}", 60, PRIVATE );
    
    IPAddress myIP = WiFi.localIP();
    Spark.publish("MY_IP", 
    String(myIP[0]) + "." + String(myIP[1]) + "." + String(myIP[2]) + "." + String(myIP[3]), 60, PRIVATE );
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
        
        //set LED to green to show scan has started
        pixel.setPixelColor(0, pixel.Color(0,255,0));
        pixel.show();
        
        scanned.clear();
        Serial.println("");
        scanner.startScan();
        
        getLocation();
    }
    else {
        if (scanning){
            WifiScanResults_t result;
            scanning = scanner.next(result);
            String name = ssid(result);
            
            if (scanning && name.length() && is_scanned_with_push(name)) {
                // already seen this name, so stop scanning
                
                //set to red when scan is done
                pixel.setPixelColor(0, pixel.Color(255,0,0));
                pixel.show();
                
                scanning = false;
            }
            else {
                 if (name != "" && gpsFixG > 0){
                    Serial.println(ssid(result) + "," + result.security + "," + result.rssi);
                    myFile = SD.open("data.txt", FILE_WRITE);
                    
                    //turn LED to blue
                    pixel.setPixelColor(0, pixel.Color(0,0,255));
                    pixel.show();
                    
                    myFile.println(gpsLatG + "," + gpsLongG + "," + ssid(result) + "," + result.security + "," + result.rssi);
                    myFile.close();
                    
                    //set back to green
                    pixel.setPixelColor(0, pixel.Color(0,255,0));
                    pixel.show();
                }
            }
        }
    }
    
}

/**
 * Determines if a vector contains a string.
 * @param v     The vector to check
 * @param value The value to look for
 * @return {@code true} if the string was found in the vector.
 */
bool contains(std::vector<String>& v, String& value) {
    return std::find(v.begin(), v.end(), value)!=v.end();
}

bool is_scanned_with_push(String& name) {
    bool result = contains(scanned, name);
    if (!result)
        scanned.push_back(name);
    return result;
}


String ssid(WifiScanResults_t& result) {
    char buf[33];
    memcpy(buf, result.ssid, 32);
    buf[result.ssidlen] = 0;
    return buf;
}

void getLocation(){
     // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   {
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
  }

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
    //if (GPS.fix) 
    //{
      Serial.print("Location: ");
      
      String gpsLat = String(convertDegMinToDecDeg(GPS.latitude, GPS.lat));
      String gpsLong = String(convertDegMinToDecDeg(GPS.longitude,  GPS.lon));
      uint8_t gpsFix = GPS.fixquality;
      
      Serial.print(gpsLat);
       Serial.print(", "); 
       Serial.println(gpsLong);
       
       gpsLatG = gpsLat;
       gpsLongG = gpsLong;
       gpsFixG = gpsFix;
      
      
}

 //http://arduinodev.woofex.net/2013/02/06/adafruit_gps_forma/
 double convertDegMinToDecDeg (float degMin, char direction) {
  double min = 0.0;
  double decDeg = 0.0;
  double directionPol = 1.0;
  
  if (direction == 'S' || direction == 'W'){
      directionPol = -1.0;
  }
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
  decDeg = decDeg * directionPol;
 
  return decDeg;
}