#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;

// Use Serial2 on ESP32 for GPS (can also use Serial1)
#define GPS_RX 16  // Connect to GPS TX
#define GPS_TX 17  // Connect to GPS RX
HardwareSerial GPSSerial(1);

// Structure to hold known location data
struct Location {
  const char* name;
  double latitude;
  double longitude;
};

// List of predefined locations
Location locations[] = {
  {"Udupi Bus Stop", 13.343034, 74.747536},
  {"Karavali Bypass", 13.346456, 74.736265},
  {"Santhekatte", 13.383474, 74.738561},
  {"SMS", 13.429662, 74.742184},
  {"Brahmavara Bus Stop", 13.436795, 74.744310},
  {"Mabukala", 13.456879, 74.712044},
  {"Sasthana", 13.469765, 74.713790},
  {"Saligrama", 13.496499, 74.710197},
  {"Kota Murkai", 13.5088249, 74.7072914},
  {"Kota", 13.5248476, 74.7057562},
  {"Tekatte", 13.5503863, 74.7013326},
  {"Kumbashi", 13.5699852, 74.6987094},
  {"Koteshwara", 13.5898806, 74.6983010},
  {"Kundapura", 13.6237621, 74.6915797}
};

const int numLocations = sizeof(locations) / sizeof(locations[0]);
const double RADIUS_METERS = 50.0;  // Detection radius in meters

void setup() {
  Serial.begin(115200);  // Start serial monitor
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // Start GPS serial
  Serial.println("GPS Location Tracker Starting...");  // Print startup message
}

void loop() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());  // Decode incoming GPS data

    if (gps.location.isUpdated()) {
      double currentLat = gps.location.lat();  // Get current latitude
      double currentLon = gps.location.lng();  // Get current longitude

      // Print current location to Serial Monitor
      Serial.print("Current Location: ");
      Serial.print(currentLat, 6);
      Serial.print(", ");
      Serial.println(currentLon, 6);

      // Compare current location with each known location
      for (int i = 0; i < numLocations; i++) {
        double distance = calculateDistance(currentLat, currentLon, locations[i].latitude, locations[i].longitude);
        if (distance < RADIUS_METERS) {
          // Print message when near a predefined location
          Serial.print("You are near: ");
          Serial.println(locations[i].name);
        }
      }

      delay(2000);  // Wait a bit before checking again
    }
  }
}

// Haversine formula to calculate distance between two lat/lon points
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Radius of Earth in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2) * sin(dLat/2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}
