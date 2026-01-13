#include <Arduino.h>

// ==================== SENSOR CALIBRATION ====================
// declare only (no assignment!)
extern const float VREF;
extern const float SENSITIVITY;
extern const float MAX_CURRENT;

// ==================== FLAGS / STATES ====================
extern bool flagAvoid;
extern bool resistanceFlag;

// ==================== OTHER CONSTANTS ====================
extern float avgCurrent;
extern const float TRIGGER_LEVEL;
extern const unsigned long FlagLatchTime;

// ==================== FUNCTION DECLARATIONS ====================
float readCurrentAmps();
void checkResistance(float normCurrent);
void ampFlag();
void plotCurrentRaw();