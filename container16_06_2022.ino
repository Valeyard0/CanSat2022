#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>
#include <DS1307RTC.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "KickMath.h"


//////////// EEPROM VARIABLES ///////////

#define software_state_address 10

#define referance_altitude_low_address 20
#define referance_altitude_high_address 21

#define mode_low_address 30
#define mode_high_address 31

#define packet_count_low_address 40
#define packet_count_high_address 41

#define TPRelease_low_address 50
#define TPRelease_high_address 51

#define calibration_state_address 60


#define cxon_com_address 70

#define median_low_address 80
#define median_high_address 81


uint8_t cxon_com = 0;
uint8_t cxoff_com = 0;
uint8_t tpx_off = 0;
uint8_t tpx_on = 0;


long median_current_time = millis();
long median_prev_time = 0;

uint16_t altitude_calc_median;
uint8_t i = 0;
uint16_t altitude_median[4] = {};


//////////// EEPROM VARIABLES ///////////

static const int RXPin = 31 , TXPin = 32;
static const uint32_t GPSBaud = 9600;

String enable_check;


#define seaLevel 1018
#define parachute_release A6
#define payload_release A7
#define Buzzer 35
#define Led 36
#define camera 2

//////  INIT ///////
Adafruit_BMP280 bmp;
Servo myservo;
TinyGPSPlus gps;
tmElements_t tm;
File myFile;
const int chipSelect = BUILTIN_SDCARD;
//////  INIT ///////

SoftwareSerial mySerial(RXPin , TXPin);

////// VOLTAGE DIVIDER VARIABLES /////
float vin;
float vout;
////// VOLTAGE DIVIDER VARIABLES /////


///////////HYPSOMETRIC FORMULA & SIMP///////////////////////////
float temp = 288.15;
float sbt = 0.0065;
float Po = 101325;
float sbt_pow = 0.1902;
double TEMP = 15;
long int SIMP;
int altitude_simp = 0;
///////////HYPSOMETRIC FORMULA & SIMP///////////////////////////


//// PAYLOAD ALTITUDE DEPEANDING VARIABLES ////
long payload_altitude_time = millis();
int payload_altitude_prev_time = 0;
//// PAYLOAD ALTITUDE DEPEANDING VARIABLES ////


//// PARACHUTE ALTITUDE DEPEANDING VARIABLES ////
long parachute_altitude_time = millis();
int parachute_altitude_prev_time = 0;
//// PARACHUTE ALTITUDE DEPEANDING VARIABLES ////



//// STATE RELEASE VARIIABLES /////
long state5_prev_altitude_prev_time = 0;
long state5_prev_altitude_time = 0;
boolean parachute_altitude_status = false;
boolean payload_altitude_status = false;
//// STATE RELEASE VARIIABLES /////


//// GENERAL STATUS /////
uint8_t calibration_state = 0;
uint8_t prev_altitude_counter = 0;
bool TPRelease_activate_tpx_release = false;
bool F_servo_status = false;
bool D_servo_status = false;
bool servo_mode = false;
bool cmd_activateRelease = false;
bool cmd_activate_tpx_release = false;
bool data_flow = false;
bool D_cmd_activate_tpx_release = false;
uint8_t mode = 1;
String char_mode = 'F';
uint8_t TP_Released = 0;
char char_TP_Released = 'N';
uint8_t flight_status = 0;
String software_state = "INITIALIZE";
long prev_altitude_time = millis();
uint32_t prev_altitude_prev_time = 0;
uint8_t int_mode = 0;
//// GENERAL STATUS /////

//// BMP VARIABLES /////
float altitude;
float temperature;
float state2_prev_altitude;
float state5_prev_altitude;
int referance_altitude = 0;
//// BMP VARIABLES /////


String relay_time;
String mission_time;
String cmd_echo;
String cmd_received[6];
long current_time = millis();
uint32_t prev_time = 0;
String package;
char packet_type = 'C';

long cmd_time = millis();
String cmd_receive = " ";
uint32_t cmd_prev_time = 0;
uint8_t servo_tick = 0;
uint16_t packet_count = 1;
String payload_data_receive = " ";

//// TIME VARIABLES /////
uint8_t hh;
uint8_t mm;
uint8_t  ss;
//// TIME VARIABLES /////

//// GPS VARIIABLES /////
float latitude;
float longitude;
float gps_altitude;
uint8_t satellite_count;
//// GPS VARIIABLES /////

long ms_current_time = millis();
long ms_prev_time = 0;

uint8_t milli = 0;

void ms_calculate() {
  ms_current_time = millis();
  if (ms_current_time - ms_prev_time >= 999) {
    milli = ms_current_time - ms_prev_time;
    ms_prev_time = ms_current_time;
    if (milli > 1000) {
      milli /= 100;
    }
  }

}


void medianTime() {
  median_current_time = millis();
  if (median_current_time - median_prev_time > 443) {
    median_prev_time = median_current_time;
    altitude_median[i++] = altitude;
    if (i == 4) {
      altitude_calc_median = KickMath<uint16_t>::calcMedian(i, altitude_median);
      i = 0;
      eeprom_write(median_low_address , median_high_address , altitude_calc_median);
    }
  }
}



void eeprom_write(int low, int high, int values)
{
  byte lowbyte = lowByte(values);
  byte highbyte = highByte(values);
  EEPROM.update(low, lowbyte);
  EEPROM.update(high, highbyte);
}


int eeprom_read(int read_low, int read_high)
{
  byte  low = EEPROM.read(read_low);
  byte high = EEPROM.read(read_high);
  int recovery = low + (high << 8);
  return recovery;
}


void cameraStart() {
  digitalWrite(camera , HIGH);
}


void cameraShutdown() {
  digitalWrite(camera, LOW);
  delay(750);
  digitalWrite(camera, HIGH);


}



void clearEEPROM() {
  for (int i = 0 ; i < 85 ; i++) {
    if (i == mode_low_address)
      continue;
    if (i == mode_high_address)
      continue;
    if (i == referance_altitude_low_address)
      continue;
    if (i == referance_altitude_high_address)
      continue;
    if (i == TPRelease_low_address)
      continue;
    if (i == TPRelease_high_address)
      continue;
    if (i == calibration_state_address)
      continue;
    if (i == cxon_com_address)
      continue;

      EEPROM.update(i, 0);
  }
}

void wipeEEPROM() {

  for (int i = 0 ; i < 85 ; i++) {

    if (i == mode_low_address)
      continue;
    if (i == mode_high_address)
      continue;
    EEPROM.update(i, 0);
  }
  packet_count = 1;
  data_flow = false;
}


void FServo() {
  if (F_servo_status == false) {
    F_servo_status = true;
    myservo.attach(22);
    servo_mode = true;
  }

  if (F_servo_status == true) {
    myservo.write(360);
  }


  if ((servo_tick > 20 ) && (F_servo_status == true)) {
    myservo.detach();
    F_servo_status = false;
    servo_tick = 0;
    servo_mode = false;
    digitalWrite(Buzzer , LOW);
  }

}

void DServo() {
  if (D_servo_status == false) {
    D_servo_status = true;
    myservo.attach(22);
    servo_mode = true;
  }

  if (D_servo_status == true) {
    myservo.write(360);

  }

  if ((servo_tick > 3) && (D_servo_status == true)) {
    myservo.detach();
    D_servo_status = false;
    servo_tick = 0;
    servo_mode = false;
    digitalWrite(Buzzer , LOW);
  }

}

void coms() {
  cmd_time = millis();
  if (Serial.available() > 0) {
    Serial.setTimeout(10);      // 20
    cmd_receive = Serial.readStringUntil(',');
    Serial.print(cmd_receive);
  }

  if (Serial3.available() > 0) {
    Serial3.setTimeout(20);         // 20
    for (uint8_t i = 0; i < 6 ; i++) {
      cmd_received[i] = Serial3.readStringUntil(',');
    }

    if (cmd_received[0] == "CMD" && cmd_received[1] == "1101" && cmd_received[2] == "CX" && cmd_received[3] == "ON") {
      if (cxon_com == 0) {
        cmd_echo = "CXON";
        data_flow = true;
        cxon_com = 1;
        EEPROM.write(cxon_com_address , cxon_com);
      }


    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "1101" && cmd_received[2] == "CX" && cmd_received[3] == "OFF") {
      cmd_echo = "CXOFF";
      data_flow = false;
      myFile.close();
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "1101" && cmd_received[2] == "SIM" && cmd_received[3] == "ENABLE") {
      cmd_echo = "ENABLE";
      enable_check = cmd_received[3];

    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "1101" && cmd_received[2] == "SIM" && cmd_received[3] == "ACTIVATE") {
      if (enable_check == "ENABLE") {
        cmd_echo = "ACTIVATE";
        mode = 0;
        char_mode = 'S';
        flight_status = 1;
        eeprom_write(mode_low_address , mode_high_address , mode);

      }

    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "1101" && cmd_received[2] == "SIM" && cmd_received[3] == "DISABLE") {
      cmd_echo = "DISABLE";
      char_mode = 'F';
      mode = 1;
      eeprom_write(mode_low_address , mode_high_address , mode);
    }

    if (cmd_received[0] == "CMD" && cmd_received[1] == "1101" && cmd_received[2] == "SIMP") {
      cmd_echo = cmd_received[3];
      SIMP = cmd_received[3].toInt();
      cmd_received[3] = " ";
      altitude_simp = (((pow(Po / SIMP, sbt_pow) - 1) * (temp)) / sbt);
    }


    if (cmd_received[0] == "CMD" && cmd_received[1] == "1101" && cmd_received[2] == "ST") {
      hh = cmd_received[3].toInt();
      mm = cmd_received[4].toInt();
      ss = cmd_received[5].toInt();
      Serial.println(hh);
      Serial.println(mm);
      Serial.println(ss);
      setTime(hh, mm, ss, 8, 5, 2022);
      RTC.set(now());
      relay_time += "CMD,";
      relay_time += "6101,";
      relay_time += "ST,";
      relay_time += hh;
      relay_time += ",";
      relay_time += mm;
      relay_time += ",";
      relay_time += ss;
      relay_time += ",";
      cmd_echo = "ST";
      Serial2.println(relay_time);
    }

    if (cmd_received[0] == "CMD" && cmd_received[1] == "6101" && cmd_received[2] == "TPX" && cmd_received[3] == "ON") {
      cmd_echo = "TPXON";
      Serial2.println("CMD,6101,TPX,ON,");
    }

    if (cmd_received[0] == "CMD" && cmd_received[1] == "6101" && cmd_received[2] == "TPX" && cmd_received[3] == "OFF") {
      cmd_echo = "TPXOFF";
      Serial2.println("CMD,6101,TPX,OFF,");

    }

    if (cmd_received[0] == "wipeEEPROM") {
      wipeEEPROM();
      cmd_received[0] = "";
    }

    if (cmd_received[0] == "CMD" && cmd_received[1] == "1101" && cmd_received[2] == "CBN") {
      cmd_echo = "CBN";
      if (calibration_state == 0) {
        calibration_state = 1;
        SD.remove("CanSat_Flight_1101_C.csv");
        EEPROM.write(calibration_state_address , calibration_state);

        if (mode == 0) {
          referance_altitude = altitude_simp;
          eeprom_write(referance_altitude_low_address , referance_altitude_high_address , referance_altitude);
        }

        if (mode == 1) {
          referance_altitude = bmp.readAltitude(seaLevel);
          eeprom_write(referance_altitude_low_address , referance_altitude_high_address , referance_altitude);
        }
        packet_count = 1;
        flight_status = 1;
        clearEEPROM();
        Serial2.println("CMD,6101,CBN,");
      }

    }

  }


  if (Serial2.available() > 0) {
    myFile = SD.open("CanSat_Flight_1101_C.csv", FILE_WRITE);
    Serial2.setTimeout(20);         // 20
    payload_data_receive = Serial2.readString();
    Serial3.println(payload_data_receive);
    myFile.println(payload_data_receive);
    payload_data_receive = "";
    myFile.close();
    //Serial2.flush();  /// GREKLİ Mİ?
    //Serial3.flush();
  }

  if (F_servo_status == true) {
    FServo();
  }
  else if (D_servo_status == true) {
    DServo();
  }

  cmd_releaseTP();
  cmd_releaseTPX();
}


void cmd_releaseTP() {
  if (cmd_received[0] == "PRelease") { // Paraşüt Ayrılması
    cmd_activateRelease = true;
    cmd_prev_time = cmd_time;
    cmd_received[0] = "";
    cmd_echo = "PRelease";
    digitalWrite(Buzzer, HIGH);
    analogWrite(parachute_release , 185);
  }

  else if ((cmd_time - cmd_prev_time > 2499) && (cmd_activateRelease == true)) {
    analogWrite(parachute_release , 0);
    cmd_activateRelease = false;
    digitalWrite(Buzzer, LOW);
  }
}



void cmd_releaseTPX() {

  if (cmd_received[0] == "FRelease") { // 25 Saniye Servo
    cmd_received[0] = "";
    TP_Released = 1;
    char_TP_Released  = 'R';
    cmd_echo = "FRelease";
    eeprom_write(TPRelease_low_address , TPRelease_high_address, TP_Released);
    //digitalWrite(Buzzer, HIGH);
    FServo();
  }

  if (cmd_received[0] == "DRelease") { // 6 Saniye Servo
    cmd_received[0] = "";
    TP_Released = 1;
    char_TP_Released = 'R';
    eeprom_write(TPRelease_low_address , TPRelease_high_address, TP_Released);
    cmd_echo = "DRelease";
    digitalWrite(Buzzer, HIGH);
    DServo();
  }

  if (cmd_received[0] == "TPRelease") {
    cmd_activate_tpx_release = true;
    cmd_prev_time = cmd_time;
    cmd_received[0] = "";
    TP_Released = 1;
    char_TP_Released = 'R';
    eeprom_write(TPRelease_low_address , TPRelease_high_address, TP_Released);
    cmd_echo = "TPRelease";
    digitalWrite(Buzzer, HIGH);

    analogWrite(payload_release , 170);
  }
  else if ((cmd_time - cmd_prev_time > 999) && (cmd_activate_tpx_release == true)) {
    analogWrite(payload_release , 0);
    cmd_activate_tpx_release = false;
    digitalWrite(Buzzer, LOW);

  }
}

void parachute_altitude_trigger() {
  parachute_altitude_time = millis();
  parachute_altitude_prev_time = parachute_altitude_time;
  parachute_altitude_status = true;
  analogWrite(parachute_release, 170);
  digitalWrite(Buzzer, HIGH);
}

void parachute_altitude_shutdown() {
  if ((parachute_altitude_time - parachute_altitude_prev_time > 2499) && (parachute_altitude_status == true)) {
    analogWrite(parachute_release, 0);
    parachute_altitude_prev_time = parachute_altitude_time;
    parachute_altitude_status = false;
    //telemetry();
    digitalWrite(Buzzer, LOW);
  }

}

void payload_altitude_trigger() {
  payload_altitude_time = millis();
  payload_altitude_prev_time = payload_altitude_time;
  analogWrite(payload_release, 170);
  payload_altitude_status = true;
  digitalWrite(Buzzer, HIGH);
}

void payload_altitude_shutdown() {
  if ((payload_altitude_time - payload_altitude_prev_time > 999) && (payload_altitude_status == true)) {
    analogWrite(payload_release, 0);
    payload_altitude_prev_time = payload_altitude_time;
    payload_altitude_status = false;
    FServo();
    //telemetry();
    digitalWrite(Buzzer, LOW);

  }
}


void dataProcess() {

  if (mode == 1) {
    altitude = bmp.readAltitude(seaLevel);
    altitude -= referance_altitude;
    temperature = bmp.readTemperature();
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    satellite_count = gps.satellites.value();
    gps_altitude = gps.altitude.meters();
    voltage();
  }

  if (mode == 0) {
    altitude = altitude_simp - referance_altitude;
    temperature = bmp.readTemperature();
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    satellite_count = gps.satellites.value();
    gps_altitude = gps.altitude.meters();
    voltage();
  }

}



void prevAltitudeCheck() {
  prev_altitude_time = millis();
  state5_prev_altitude_time = millis();
  if (mode == 1) {
    if (flight_status == 2 || flight_status == 3 || flight_status == 4) {
      if (prev_altitude_time - prev_altitude_prev_time > 450) {
        state2_prev_altitude = altitude;
        prev_altitude_prev_time = prev_altitude_time;
      }
    }

  }
  if (mode == 0) {
    if (flight_status == 2 || flight_status == 3 || flight_status == 4) {
      if (prev_altitude_time - prev_altitude_prev_time > 450) {
        state2_prev_altitude = altitude;
        prev_altitude_prev_time = prev_altitude_time;
      }
    }
    if (flight_status == 5 || flight_status == 6) {
      if (state5_prev_altitude_time - state5_prev_altitude_prev_time > 499) {
        state5_prev_altitude = altitude;
        state5_prev_altitude_prev_time = state5_prev_altitude_time;
      }

    }
  }
}

String printTime(TinyGPSTime & t) {
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());
  return sz;
}

void voltage() {
  vin = (analogRead(A20) * 4.2000) / 1024;
  vout = ((vin)) * ((0.89));
}



void telemetry() {
  myFile = SD.open("CanSat_Flight_1101_C.csv", FILE_WRITE);
  package += "1101,";
  if (hour() < 10)
    package += "0";
  package += hour();
  package += ":";
  if (minute() < 10)
    package += "0";
  package += minute();
  package += ":";
  if (second() < 10)
    package += "0";
  package += second();
  package += ":";
  package += milli;
  package += ",";
  package += packet_count;
  package += ",";
  package += packet_type;
  package += ",";
  package += char_mode;
  package += ",";
  package += char_TP_Released;
  package += ",";
  package += altitude;
  package += ",";
  package += temperature;
  package += ",";
  package += vout;
  package += ",";
  package += printTime(gps.time);
  package += ",";
  package += "41.4495";
  package += ",";
  package += "31.7588";
  package += ",";
  package += gps_altitude;
  package += ",";
  package += satellite_count;
  package += ",";
  package += software_state;
  package += ",";
  package += cmd_echo;
  package += ",";
  myFile.println(package);
  Serial3.println(package);

  if (flight_status == 5 || flight_status == 6) {
    if ((int)altitude == (int)state5_prev_altitude  && altitude < 40)
      prev_altitude_counter++;

    if (prev_altitude_counter > 7)
      data_flow = false;
  }

  myFile.close();
  package = "";

  eeprom_write(packet_count_low_address, packet_count_high_address, packet_count);
  packet_count++;

}
/*
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

#define TIME_HEADER "T" // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}
*/


void setup() {

  Serial.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);
  SD.begin(chipSelect);
  myservo.attach(22);
  pinMode(camera , OUTPUT);
  digitalWrite(camera , LOW);
//  setSyncProvider(getTeensy3Time);
  if (eeprom_read(mode_low_address , mode_high_address) == 0) {
    mode = 0;
    char_mode = 'S';
  }
  else {
    mode = 1;
    char_mode = 'F';
  }

  Serial3.println("Son Kalinan Mode:");
  Serial3.println(eeprom_read(mode_low_address , mode_high_address));

  Serial3.println("Son Medyan");
  Serial3.println(eeprom_read(median_low_address , median_high_address));
  altitude_calc_median = eeprom_read(median_low_address , median_high_address);

  Serial3.println("Son Alinan Packet Sayisi:");
  Serial3.println(eeprom_read(packet_count_low_address , packet_count_high_address));

  if (eeprom_read(packet_count_low_address , packet_count_high_address) > 0 )
    packet_count = eeprom_read(packet_count_low_address, packet_count_high_address);


  Serial3.println("Son Software State:");
  Serial3.println(EEPROM.read(software_state_address));
  flight_status = EEPROM.read(software_state_address);

  Serial3.println("Son Referance Altitude");
  Serial3.println(eeprom_read(referance_altitude_low_address , referance_altitude_high_address));
  referance_altitude = eeprom_read(referance_altitude_low_address , referance_altitude_high_address);

  Serial3.println("Son TPReleased Durumu");
  Serial3.println(eeprom_read(TPRelease_low_address, TPRelease_high_address));


  Serial3.println("Son Calibration Durumu");
  Serial3.println(EEPROM.read(calibration_state_address));
  calibration_state = EEPROM.read(calibration_state_address);

  Serial3.println("SON CX Durumu");
  Serial3.println(EEPROM.read(cxon_com_address));
  cxon_com = EEPROM.read(cxon_com_address);

  if (cxon_com == 1) {
    data_flow = true;
  }

  if (flight_status == 0)
    software_state = "INITIALIZE";

  if (flight_status == 1 && altitude > 20)
    software_state = "LAUNCH";


  if (flight_status == 2 && (altitude - state2_prev_altitude < 0))
    software_state = "DESCENT";

  if (flight_status == 3 && (altitude < 420)  && (altitude_calc_median < 420))
    software_state = "PARACHUTE";

  if (flight_status == 4 && (altitude < 310) && (altitude_calc_median < 310))
    software_state = "RELEASE";

  if (flight_status == 5 &&  prev_altitude_counter > 5)
    software_state = "LANDING";

  if (eeprom_read(TPRelease_low_address , TPRelease_high_address) == 0)
    char_TP_Released = 'N';


  if (eeprom_read(TPRelease_low_address , TPRelease_high_address) != 0)
    char_TP_Released = 'R';

  myFile = SD.open("CanSat_Flight_1101_C.csv", FILE_WRITE);
  mySerial.begin(GPSBaud);
  bmp.begin(0X76 , BMP280_CHIPID);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     //* Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time.

  pinMode(parachute_release, OUTPUT);
  pinMode(payload_release, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(Led , OUTPUT);
  pinMode(A20, INPUT);
  digitalWrite(Led , HIGH);
  digitalWrite(Buzzer , HIGH);
  delay(750);
  digitalWrite(Buzzer , LOW);
  cameraStart();

  
}
void loop() {
/*
  time_t t = processSyncMessage();
  if (t != 0) {
    Teensy3Clock.set(t); // set the RTC
    setTime(t);
  }

*/
  ms_calculate();
  medianTime();
  prevAltitudeCheck();
  coms();
  payload_altitude_time = millis();
  parachute_altitude_time = millis();
  current_time = millis();
  dataProcess();
  payload_altitude_shutdown();
  parachute_altitude_shutdown();

  if (flight_status == 1 && altitude > 20) {
    flight_status = 2;
    software_state = "LAUNCH";
    dataProcess();
    EEPROM.update(software_state_address , flight_status);

  }

  if ((flight_status == 2) && (altitude - state2_prev_altitude < 0) && (altitude_calc_median > 550)) {
    flight_status = 3;
    EEPROM.update(software_state_address , flight_status);
    software_state = "DESCENT";
    dataProcess();
  }

  if (flight_status == 3 && (altitude < 420) && (altitude_calc_median < 420)) {
    flight_status = 4;
    EEPROM.update(software_state_address , flight_status);
    parachute_altitude_trigger();
    software_state = "PARACHUTE";
  }

  if (flight_status == 4 && (altitude < 310) && (altitude_calc_median < 310)) {
    flight_status = 5;
    EEPROM.update(software_state_address , flight_status);
    payload_altitude_trigger();
    char_TP_Released = 'R';
    TP_Released = 1;
    eeprom_write(TPRelease_low_address , TPRelease_high_address, TP_Released);
    software_state = "RELEASE";
    Serial2.println("CMD,6101,TPX,ON,");
  }


  if (flight_status == 5 && prev_altitude_counter > 6) {
    Serial2.println("CMD,6101,TPX,OFF,");
    digitalWrite(Buzzer, HIGH);
    software_state = "LANDING";
    cameraShutdown();
    flight_status = 6;

  }

  if ((current_time - prev_time > 999 ) && (servo_mode == true)) {
    if (D_servo_status == true) {
      servo_tick++;
      prev_time = current_time;
    }

    if (F_servo_status == true) {
      servo_tick++;
      prev_time = current_time;
    }
    if (data_flow == true)
      telemetry();

  }
  if ((current_time - prev_time > 993) && (data_flow == true)) {
    prev_time = current_time;
    telemetry();

  }

}
