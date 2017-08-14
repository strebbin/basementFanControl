#include "dht.h"
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins


// pin 0..1 for flashing / serial
#define INDOOR_SENSOR_PIN 2
#define OUTDOOR_SENSOR_PIN 3
// pin 5..10 for lcd
#define LCD_BACKLIGHT_PIN 6
#define FANPWM_PIN 5
// pin 12 not in use
#define ERROR_LED_PIN 13

#define FAN_SPEED_CONTROL_PIN A0
// A1..A2 not in use at the moment
//#define MOTION_PIN A3
#define OUTPUT_PIN1 A4
//#define RELAY_PIN2 A5
// A6..A7 not in use at the moment INPUT ONLY!

#define INDOOR_SENSOR_TYPE "DHT11"
#define OUTDOOR_SENSOR_TYPE "DHT11"

#define MEASUREMENT_INTERVAL_MINUTES 5
#define ERROR_INTERVAL_SECONDS 10

#define MIN_INDOOR_TEMPERATURE 10
#define MIN_OUTDOOR_TEMPERATURE -1

// Genauigkeit der Temperaturmessung in +-°C
#define INDOOR_SENSOR_PRECISION_TEMPERATURE 2
#define OUTDOOR_SENSOR_PRECISION_TEMPERATURE 2

// Genauigkeit der Feuchtemessung in +- %
#define INDOOR_SENSOR_PRECISION_HUMIDITY 5
#define OUTDOOR_SENSOR_PRECISION_HUMIDITY 5

const int rs = 12, en = 11, d4 = 10, d5 = 9, d6 = 8, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

dht IndoorSensor;
dht OutdoorSensor;

double maxOutdoorDewPoint;
double minIndoorDewPoint;

bool fanIsOn = false;
int fanSpeedControllSensorValue = 0;

void setup()
{
  // set up the LCD's number of columns and rows:
  // lcd.begin(16, 2);
  Serial.begin(115200);
  pinMode(OUTPUT_PIN1, OUTPUT);
  //  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(FANPWM_PIN, OUTPUT);
  //  pinMode(MOTION_PIN, INPUT);
  pinMode(FAN_SPEED_CONTROL_PIN, INPUT);

  digitalWrite(OUTPUT_PIN1, LOW);
  digitalWrite(OUTPUT_PIN1, INPUT_PULLUP);
  //  digitalWrite(RELAY_PIN2, HIGH);

  Serial.println("Type,\tHumidity (%),\tTemperature (C),\tDew Point");

  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  //lcd.backlight();
  lcd.begin(16, 2);              // initialize the lcd
  lcd.print("Entfeuchtung");
  delay(3000);
}

void OutputReturnStatus( int ret )
{
  switch (ret)
  {
    case DHTLIB_OK:
      Serial.print("OK,\t");
      break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.print("Checksum error,\t");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.print("Time out error,\t");
      break;
    default:
      Serial.print("Unknown error,\t");
      break;
  }
}

int getFanPWMValue() {
  fanSpeedControllSensorValue = analogRead(FAN_SPEED_CONTROL_PIN);
  return map(fanSpeedControllSensorValue, 0, 1023, 0, 255);
}

void SetFanPWM()
{
  analogWrite(FANPWM_PIN, getFanPWMValue());
}

void OutputValues(dht sensor)
{
  Serial.print(sensor.humidity, 1);
  Serial.print(",\t");
  Serial.print(sensor.temperature, 1);
  Serial.print(",\t");
  Serial.println(getDewPoint(sensor.temperature, sensor.humidity), 1);
  //Serial.print(",\t");
  //Serial.println(getDewPointSlow(sensor.temperature, sensor.humidity), 1);
}
uint16_t i = 0;

void loop()
{
  //bool isLCDEnabled = digitalRead(MOTION_PIN) == HIGH;
  //if (isLCDEnabled) {
  //  lcd.setBacklight(255);
  //  } else {
  //  lcd.setBacklight(10);
  //}

  SetFanPWM();

  if (i == 0) {
    bool measureCorrect = measureAndCalculate();
    if (!measureCorrect) {
      i = 0;
      digitalWrite(ERROR_LED_PIN, HIGH);
      return;
    }

    digitalWrite(ERROR_LED_PIN, LOW);
    SetOutput();
    displayValues();
  }

  i++;
  if (i >= MEASUREMENT_INTERVAL_MINUTES * 60L * 10L)  {
    i = 0;
  }

  delay(100L);
}

void ActivateFan() {
  Serial.println("Fan ON");
  if (!fanIsOn) {
    analogWrite(FANPWM_PIN, 0);
    digitalWrite(OUTPUT_PIN1, HIGH);
    //    digitalWrite(RELAY_PIN2, LOW);

    // slow start of fan to reduce current
    int maxPWM = getFanPWMValue();
    Serial.print("Start fan to ");
    Serial.println(maxPWM);
    for (int i = 0; i < maxPWM; i++) {
      analogWrite(FANPWM_PIN, i);
      delay(50);
      Serial.print(i);Serial.print(" ");
    }
    fanIsOn = true;
  }
}

void DeactivateFan() {
  // Stoppe Luefter
  Serial.println("Fan OFF");
  fanIsOn = false;
  digitalWrite(OUTPUT_PIN1, LOW);
  //    digitalWrite(RELAY_PIN2, HIGH);
}

void SetOutput() {
  if (IndoorSensor.temperature > MIN_INDOOR_TEMPERATURE &&
      OutdoorSensor.temperature > MIN_OUTDOOR_TEMPERATURE &&
      maxOutdoorDewPoint < IndoorSensor.temperature &&
      maxOutdoorDewPoint < minIndoorDewPoint) {
    // Aktiviere Luefter
    ActivateFan();
  } else {
    DeactivateFan();
  }
}

// DISPLAY DATA
void displayValues() {

  lcd.clear();

  //##################
  //#O: 16.2 80% 17.3#
  //#I: 18.3 70% 13.4#
  //##################

  lcd.home();
  lcd.print("O: ");
  lcd.print(OutdoorSensor.temperature, 1);
  lcd.setCursor(8, 0);
  lcd.print(OutdoorSensor.humidity, 0);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("I: ");
  lcd.print(IndoorSensor.temperature, 1);
  lcd.setCursor(8, 1);
  lcd.print(IndoorSensor.humidity, 0);
  lcd.print("%");

  if (OutdoorSensor.temperature <= MIN_OUTDOOR_TEMPERATURE) {
    lcd.setCursor(12, 0);
    lcd.print("low");

    if (IndoorSensor.temperature <= MIN_INDOOR_TEMPERATURE) {
      lcd.setCursor(12, 1);
      lcd.print("low");
    }
  }

  int sensorpercentage = map(fanSpeedControllSensorValue, 0, 1023, 0, 100);
  if (IndoorSensor.temperature > MIN_INDOOR_TEMPERATURE && OutdoorSensor.temperature > MIN_OUTDOOR_TEMPERATURE)
  {
    lcd.setCursor(12, 0);
    lcd.print(fanIsOn ? "ON" : "OFF");

    lcd.setCursor(12, 1);

    lcd.print(sensorpercentage);
    lcd.print("%");
  }

  Serial.print("OUTDOOR, \t");
  OutputValues(OutdoorSensor);

  Serial.print("INDOOR, \t");
  OutputValues(IndoorSensor);

  // Debuggingwerte
  Serial.print("maxOutDewPoint: \t");
  Serial.print(maxOutdoorDewPoint, 1);
  Serial.print("\t<\t");
  Serial.print("\tminInDewPoint: \t");
  Serial.println(minIndoorDewPoint, 1);
  Serial.print("Fan is: ");
  Serial.print(fanIsOn ? "ON at " : "OFF at ");
  Serial.print(sensorpercentage);
  Serial.println("%");
}

bool measureAndCalculate() {

  // READ DATA
  int ret_out = INDOOR_SENSOR_TYPE == "DHT11" ? OutdoorSensor.read11(OUTDOOR_SENSOR_PIN) : OutdoorSensor.read(OUTDOOR_SENSOR_PIN);
  int ret_in = INDOOR_SENSOR_TYPE == "DHT11" ? IndoorSensor.read11(INDOOR_SENSOR_PIN) : IndoorSensor.read(INDOOR_SENSOR_PIN);

  if (ret_out != DHTLIB_OK || ret_in != DHTLIB_OK) {
    OutputReturnStatus(ret_out);
    OutputReturnStatus(ret_in);
    lcd.clear();
    // set the cursor to (0,0):
    lcd.home();
    lcd.print("Sensor Error:");
    lcd.setCursor(0, 1);
    if (ret_out != DHTLIB_OK) {
      lcd.print("OUTDOOR ");
    }
    if (ret_out != DHTLIB_OK && ret_in != DHTLIB_OK) {
      lcd.print("+ ");
    }
    if (ret_in != DHTLIB_OK) {
      lcd.print("INDOOR");
    }
    Serial.println("");
    // Stoppe Luefter
    fanIsOn = false;
    digitalWrite(OUTPUT_PIN1, LOW);
    //    digitalWrite(RELAY_PIN2, HIGH);
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(ERROR_INTERVAL_SECONDS * 1000);
    return false;
  }

  maxOutdoorDewPoint = getDewPoint(OutdoorSensor.temperature + OUTDOOR_SENSOR_PRECISION_TEMPERATURE, OutdoorSensor.humidity + OUTDOOR_SENSOR_PRECISION_HUMIDITY);
  minIndoorDewPoint = getDewPoint(IndoorSensor.temperature - INDOOR_SENSOR_PRECISION_TEMPERATURE, IndoorSensor.humidity - INDOOR_SENSOR_PRECISION_HUMIDITY);
  return true;
}


// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double getDewPoint(double temperature, double humidity) {
  double a = 17.271;
  double b = 237.7;
  double temp_ = (a * (double) temperature) / (b + (double) temperature) + log( (double) humidity / 100);
  double Td = (b * temp_) / (a - temp_);
  return Td;

}
// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm
double getDewPointSlow(double temperature, double humidity) {
  double A0 = 373.15 / (273.15 + (double) temperature);
  double SUM = -7.90298 * (A0 - 1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1) ;
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * (double) humidity;
  double T = log(VP / 0.61078); // temp var
  return (241.88 * T) / (17.558 - T);
}

