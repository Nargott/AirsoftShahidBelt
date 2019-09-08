#include <Arduino.h>
#include "GyverTM1637.h"
#include <TimerOne.h>

#include "ShiftIn.h"
 //MOSI: pin 11 don't use for anything
 //MISO: pin 12 to Q7 of last register
 //SCK:  pin 13 to CP
 // PL:  pin 2

#define DISP_CLK 6
#define DISP_DIO 5

#define SW_PIN_1 9
#define SW_PIN_2 10
#define SW_PIN_3 4

#define SHIFT_PL 2
#define SHIFT_CE 11
#define SHIFT_DATA 12
#define SHIFT_CP 13

#define LED_RED_PIN 7
#define LED_GREEN_PIN 8

#define RELAY_PIN 3

GyverTM1637 disp(DISP_CLK, DISP_DIO);

ShiftIn<2> shift;

#define DISARM_WIRES_COUNT 5
byte disarm[8][DISARM_WIRES_COUNT] = {
  {2, 0, 6, 9, 8},
  {10, 4, 2, 11, 5},
  {8, 14, 10, 3, 6},
  {11, 5, 12, 4, 10},
  {12, 9, 6, 0, 14},
  {6, 10, 8, 0, 3},
  {0, 13, 4, 9, 11},
  {12, 9, 3, 1, 14}
};

byte variant = 0;
unsigned int startTime = 0;
uint64_t lastUpdatedAt = 0;

enum STATES {
  STATE_CONFIG,
  STATE_ARMED,
  STATE_DISARMED,
  STATE_BOOM,
} state;

void setVariantFromSWValue() {
  byte v = 0;
  v |= digitalRead(SW_PIN_1) << 0;   
  v |= digitalRead(SW_PIN_2) << 1;
  v |= digitalRead(SW_PIN_3) << 2;
  variant = v;

  return;
}

void timerEvent() {
  if (state == STATE_ARMED) startTime--;
}

void setState(STATES newState) {
  //Serial.println("NewState = " + String(newState));
  switch (newState) {
  case STATE_CONFIG:
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    state = STATE_CONFIG;
    break;
  case STATE_ARMED:
    digitalWrite(RELAY_PIN, LOW);
    Timer1.initialize(1000000);
    Timer1.attachInterrupt(timerEvent); 
    state = STATE_ARMED;
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    break;
  case STATE_BOOM: 
    digitalWrite(RELAY_PIN, HIGH);
    Timer1.detachInterrupt();
    state = STATE_BOOM;
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    break;
  case STATE_DISARMED: 
    digitalWrite(RELAY_PIN, LOW);
    Timer1.detachInterrupt();
    state = STATE_DISARMED;
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    break;
  default:
    break;
  }
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  setState(STATE_CONFIG);

  //Serial.begin(9600);

  pinMode(SW_PIN_1, INPUT);
  pinMode(SW_PIN_2, INPUT);
  pinMode(SW_PIN_3, INPUT);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  shift.begin(SHIFT_PL, SHIFT_CE, SHIFT_DATA, SHIFT_CP);

  disp.clear();
  disp.brightness(7);
}

/* isIndexInSelectedVariant returns true if 
  this index in the selected variant array
*/
bool isIndexInSelectedVariant(uint16_t idx) {
  for (uint8_t i = 0; i < DISARM_WIRES_COUNT; i++) {
    if (idx == disarm[variant][i]) return true;
  }

  return false;
}

/* readWires returns count of disconnected disarm wires
  or returns -1 if wrong wire disconnected
  or 0 if all wires are connected
*/
int readWires() {
  int result = 0;
  shift.update();
  for(uint16_t i = 0; i < shift.getDataWidth() - 1; i++) {//the last one is not used
    //Serial.print(shift.state(i));
    if (shift.state(i) == 1) {
      if (isIndexInSelectedVariant(i)) {result++;}
      else { //wrong wire disconnected
        //Serial.println();
        return -1;
      }
    }
  }	
  //Serial.println();
  return result;
}

void configuration() {
  if (millis() - lastUpdatedAt > 1000) {
    setVariantFromSWValue();
    startTime = map(analogRead(0), 0, 1023, 1, 90) * 60;
    disp.clear();
    byte mins = floor(startTime/60);
    byte secs = startTime - mins*60;
    disp.displayClock(mins, secs);
    disp.point(true);
    if (readWires() == 0) setState(STATE_ARMED);
    lastUpdatedAt = millis();
  }
}

bool displayPoint = true;
void armed() {
  if (millis() - lastUpdatedAt > 1000) {
    byte mins = floor(startTime/60);
    byte secs = startTime - mins*60;
    disp.displayClock(mins, secs);
    disp.point(displayPoint);
    displayPoint = !displayPoint;
    lastUpdatedAt = millis();
  }
  if ((readWires() == -1) || (startTime == 0)) setState(STATE_BOOM);
  if (readWires() == DISARM_WIRES_COUNT) setState(STATE_DISARMED);
}

void disarmed() {
  if (millis() - lastUpdatedAt > 1000) {
    byte mins = floor(startTime/60);
    byte secs = startTime - mins*60;
    disp.displayClock(mins, secs);
    disp.point(true);
    lastUpdatedAt = millis();
  }
}

bool ledBlink = HIGH;
void boom() {
  if (millis() - lastUpdatedAt > 1000) {
    byte mins = floor(startTime/60);
    byte secs = startTime - mins*60;
    disp.displayClock(mins, secs);
    disp.point(true);
    digitalWrite(LED_RED_PIN, ledBlink);
    (ledBlink == HIGH) ? ledBlink = LOW : ledBlink = HIGH;
    lastUpdatedAt = millis();
  }
}

void loop() {
  // disp.displayClockTwist(mins, secs, 10);
  switch (state) {
  case STATE_CONFIG:
    configuration();
    break;
  case STATE_ARMED:
    armed();
    break;
  case STATE_DISARMED:
    disarmed();
    break;
  case STATE_BOOM:
    boom();
    break;
  
  default:
    break;
  }
}