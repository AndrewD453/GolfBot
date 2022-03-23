#include <NMEAGPS.h>
#define NEOGPS_USE_SERIAL1
#include <GPSport.h>

NMEAGPS gps;
gps_fix fix;
NeoGPS::Location_t target;

float distance;
float myAngle;
float angle;

double getAngle(double x, double y) {
  return (map(atan2(y, x) * 180 / PI, 180, -180, 0, 360) + 90) % 360;
}

void readGPS() {
  while (gps.available(gpsPort)) { //UPDATE GPS !!!
    fix = gps.read();
  }
}
