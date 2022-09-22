/*
   Remington Frank: Intro. to Physical Computing

   RGB Sensor code sourced from the Adafruit TCS34725
   RGB Sensor example code. For further reference,
   refer to the following:

   https://learn.adafruit.com/adafruit-color-sensors/arduino-code

   VL53L0X sensor code is sourced from the Pololu library. This
   library can be found at the following link:

   https://github.com/pololu/vl53l0x-arduino

*/

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

#define echoPin 2
#define trigPin 3

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int SERV = 9;
int FWD = 6;
int REV = 7;

// Initialize evaluation criteria and other values

double hue, saturation, value;
long duration;
double distance;

float prevRunTime;
float runTime;

unsigned long trueTime;
unsigned long currTime;
unsigned long delayTime;
unsigned long printTime;

int turn;
int prevTurn;
int counter;

int h_print;
int la_print;
int dist_print;
int s_print;

bool printer = 1;

void setup(void) {
  Serial.begin(115200);
  Wire.begin();

  lcd.begin();
  lcd.backlight();

  bool start = 1;

  // Begin color sensing sensor
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  // Initilize pin modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(FWD, OUTPUT);
  pinMode(REV, OUTPUT);

  digitalWrite(FWD, LOW);
  digitalWrite(REV, HIGH);
  delay(3000);

  myServo.attach(SERV);

  sensor.setAddress(0x52);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

void loop(void) {
  trueTime = millis();
  
  uint16_t r, g, b, c, colorTemp, lux;

  // Gather various color data from RGB sensor
  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  // RGB to HSV conversion
  double rWeight = r / 65535.0;
  double gWeight = g / 65535.0;
  double bWeight = b / 65535.0;

  Serial.println("Red Weight: "); Serial.print(rWeight); Serial.println(" ");

  // RGB to HSV algorithm
  double cmax = max(rWeight, max(gWeight, bWeight)); // maximum of r, g, b
  double cmin = min(rWeight, min(gWeight, bWeight)); // minimum of r, g, b
  double diff = cmax - cmin; // diff of cmax and cmin.
  double h = -1;

  // if cmax and cmax are equal then h = 0
  if (cmax == cmin) {
    h = 0;
  }

  // if cmax equal r then compute h
  else if (cmax == rWeight) {
    h = fmod(60 * ((gWeight - bWeight) / diff) + 360, 360);
  }

  // if cmax equal g then compute h
  else if (cmax == gWeight) {
    h = fmod(60 * ((bWeight - rWeight) / diff) + 120, 360);
  }

  // if cmax equal b then compute h
  else if (cmax == bWeight) {
    h = fmod(60 * ((rWeight - gWeight) / diff) + 240, 360);
  }

  Serial.println(" ");
  Serial.print("Hue: "); Serial.print(h); Serial.print(" ");
  Serial.println(" ");
  //--------------------------------------------------------------------------------------------------------

  // Map Hue value to linear actuator distance extended
  runTime = map(h, 0, 360, 0, 3000);

  if(runTime > prevRunTime) {
    digitalWrite(FWD, HIGH);
    digitalWrite(REV, LOW);
    delay(runTime - prevRunTime);
  } 
  else if (runTime < prevRunTime){
    digitalWrite(FWD, LOW);
    digitalWrite(REV, HIGH);
    delay(prevRunTime - runTime);
  }

  prevRunTime = runTime;
  digitalWrite(FWD, LOW);
  digitalWrite(REV, LOW);

  //--------------------------------------------------------------------------------------------------------
  // Read distance extended of lin act by the ultrasonic sensor
  /*digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.println(" ");
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" ");

  delay(100);*/
  distance = sensor.readRangeContinuousMillimeters() - 63;
  Serial.print(distance);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  //--------------------------------------------------------------------------------------------------------
  // Map distance detected into 90° rotation of servo

  turn = map(distance, 0, 50, 0, 90);
  //myServo.write(-prevTurn);
  //delay(10);
  if(turn > 90) turn = 90;
  myServo.write(turn);

  prevTurn = turn;

  //--------------------------------------------------------------------------------------------------------
  // Print inputs and outputs to LCD display
  /*if(trueTime - printTime >= 2000){
    if(counter == 3) counter = 0;
    counter++;
    lcd.clear();
    printer = !printer;
    printTime = trueTime;
  }

  if(counter == 1){
    lcd.print(String("R: ") + String(r) + String(" G: ") + String(g) + String(" B: ") + String(b)); 
  }
  else if(counter == 3){
    lcd.print(String("Degrees: ") + String(turn)); 
  } else if(counter == 2){
    lcd.print(String("Distance: ") + String(distance));
  }*/

  h_print = map(h, 0, 360, 0, 99);
  la_print = map(runTime, 0, 3000, 0, 99);
  dist_print = map(distance, 0, 50, 0, 99);
  s_print = map(turn, 0, 90, 0, 99);

  if(trueTime - printTime >= 250){
    lcd.home();
    lcd.print(String("i:") + String(int(h_print)));
    lcd.setCursor(6,0);
    lcd.print(String("m:") + String(la_print));
    lcd.setCursor(8, 1);
    lcd.print(String(dist_print));
    lcd.setCursor(12, 1);
    lcd.print(String("o:") + String(s_print)); 
  }
}
