#include <KY040.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include "U8glib.h"

#define ENC_CLK_PIN 2
#define ENC_DT_PIN 3

#define GPS_RX_PIN 5
#define GPS_TX_PIN 4
#define GPS_BAUD_RATE 9600

#define SPEAKER_DATA_PIN 9
#define SPEAKER_POWER_PIN 6

const uint8_t _sl = 17;
const uint8_t _st = 5;
const uint8_t _sh = 17;

const uint8_t _digits[] PROGMEM = {
  0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110,
  0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111
};

static const uint8_t _satelliteIcon[] PROGMEM =   { 
  0x0, 0x0, 0x8, 0x0, 0x18, 0x1c, 0x3a, 0x1c, 0x7, 0x1c, 0xf, 0xa0, 0x7, 0xc0, 0x3, 0xe0,
  0x1, 0xe4, 0x2, 0xcc, 0x4, 0x18, 0x38, 0x32, 0x38, 0x66, 0x38, 0x0, 0x0, 0x0, 0x0, 0x0 };

static const unsigned char PROGMEM _satelliteIconL[] = { 
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3, 0x0, 
  0x0, 0x0, 0x7, 0x80, 0x0, 0x30, 0xf, 0xc0, 0x0, 0x70, 0x1f, 0xe0, 0x0, 0xf0, 0x3f, 0xe0, 
  0x1, 0xfc, 0x3f, 0xc0, 0x3, 0xfe, 0x1f, 0x80, 0x7, 0xff, 0x3f, 0x0, 0x7, 0xff, 0xf6, 0x0, 
  0x0, 0xff, 0xe0, 0x0, 0x0, 0xff, 0xe0, 0x0, 0x0, 0x7f, 0xe0, 0x0, 0x0, 0x3f, 0xe0, 0x0,
  0x0, 0x1f, 0xc7, 0xc0, 0x0, 0x1f, 0x9f, 0x80, 0x1, 0xbf, 0x3f, 0x0, 0x3, 0xf0, 0x7e, 0x8,
  0x7, 0xe0, 0x7c, 0x20, 0xf, 0xf0, 0xf8, 0x0, 0x1f, 0xf0, 0xf0, 0x0, 0x1f, 0xe0, 0xe0, 0xa0, 
  0xf, 0xc0, 0xc1, 0x28, 0x7, 0x80, 0x80, 0x40, 0x3, 0x0, 0x9, 0x90, 0x0, 0x0, 0x0, 0x20, 
  0x0, 0x0, 0x10, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

static const uint16_t _melody[] = { 440, 494, 523, 659, 784, 1046 };
static const uint8_t _noteDurations[] = { 8, 8, 8, 8, 8, 4 };

U8GLIB_SSD1306_128X64 display(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);
KY040 rotaryEncoder(ENC_CLK_PIN, ENC_DT_PIN);
TinyGPSPlus gps;
SoftwareSerial softSerial(GPS_RX_PIN, GPS_TX_PIN);
Adafruit_MPU6050 mpu;
//CRGB leds[NUM_LEDS];

volatile int encoder_value = 0;
char _internalCharBuf[12];

// state variables
String pageTitle = "";
uint32_t satelliteCount = 8;
uint8_t timeHour = 8;
uint8_t timeMinutes = 8;
uint16_t speed = 888;
String cardinal = "-";
double startLat = -1;
double startLng = -1;
double distanceKm;
sensors_event_t accEvent, gyroEvent, tempEvent;

// ui state
static const uint8_t HOME_SCREEN = 0;
static const uint8_t ACCELARATION_SCREEN = 1;
static const uint8_t GYRO_SCREEN = 2;
static const uint8_t LOCATION_SCREEN = 3;
static const uint8_t COURSE_SCREEN = 4;

uint8_t currentUI = HOME_SCREEN;

// ISR to handle the interrupts for CLK and DT
void ISR_RotaryEncoder() {
  switch (rotaryEncoder.getRotation()) {   
    case KY040::CLOCKWISE:
      encoder_value++;
      if (encoder_value > 4) encoder_value = 0;
      break;
    case KY040::COUNTERCLOCKWISE:
      encoder_value--;
      if (encoder_value < 0) encoder_value = 4;
      break;
  }
  //_beep(1);
}

void setup() {
  initDisplay();
  initIMU();
  pinMode(SPEAKER_POWER_PIN, OUTPUT);
  pinMode(SPEAKER_DATA_PIN, OUTPUT);
  //FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);

  // Set interrupts for ENC_CLK_PIN and ENC_DT_PIN
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), ISR_RotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), ISR_RotaryEncoder, CHANGE);

  // init gps communication
  softSerial.begin(GPS_BAUD_RATE);

  _playStartUpTone();
}

void loop() {
  readGPSData();
  readIMUData();

  // Render display  
  display.firstPage();  
  do {
    render();
  } while( display.nextPage() );
  delay(100);
}

void initDisplay() {
  display.setFont(u8g_font_6x10);
  display.setFontRefHeightExtendedText();
  display.setDefaultForegroundColor();
  display.setFontPosTop();
}

void initIMU() {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void render() {
  currentUI = encoder_value;
  _showSatelliteCount(satelliteCount);
  _showTopBar();
  _showTime(timeHour, timeMinutes);

  switch (currentUI) {
    case HOME_SCREEN: showHomeScreen(); break;
    case ACCELARATION_SCREEN: showAccelerationScreen(); break;
    case GYRO_SCREEN: showGyroScreen(); break;
    case LOCATION_SCREEN: showLocationScreen(); break;
    case COURSE_SCREEN: showCourseScreen(); break;
  }
}

void showHomeScreen() {
  pageTitle = "";
  if (satelliteCount == 0) {
      _noSatellite();
      return;
  }

  _showValueSegment(speed);
}

void showAccelerationScreen() {
  pageTitle = "ACCEL.";
  _showAccData();
}

void showGyroScreen() {
  pageTitle = "GYRO";
  _showGyroData();
}

void showLocationScreen() {
  pageTitle = "LOCATION";
  _showLocationData();
}

void showCourseScreen() {
  pageTitle = "COURSE";
  _showCourseData();
}

void readGPSData() {
  while (softSerial.available() > 0) {
    if (gps.encode(softSerial.read())) {
      
      if (gps.satellites.isValid()) {
        satelliteCount = gps.satellites.value();
      } else {
        satelliteCount = 0;
      }

      if(gps.time.isValid()) {
        timeHour = gps.time.hour();
        timeMinutes = gps.time.minute();
      } else {
        timeHour = 0;
        timeMinutes = 0;
      }

      if (gps.speed.isValid() & gps.speed.age() < 2000) {
        speed = gps.speed.kmph() * 10;
      } else {
        speed = 0;
      }

      cardinal = gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "---";

      if (gps.location.isValid()) {
        if (startLat == -1 | startLng == -1) {
          startLat = gps.location.lat();
          startLng = gps.location.lng();
        }

        distanceKm = TinyGPSPlus::distanceBetween(
          gps.location.lat(), gps.location.lng(), startLat, startLng) / 1000.0;
      }
    }
  }
}

void readIMUData() {
  mpu.getEvent(&accEvent, &gyroEvent, &tempEvent);
}

void _showSatelliteCount(uint8_t count) {
  display.drawBitmapP(0, 0, 2, 16, _satelliteIcon);
  display.setPrintPos(16, 0);
  display.print(count);
}

void _showTopBar() {
  display.drawRFrame(28, 0, 66, 15, 3);
  
  if (pageTitle != "") {
    display.setPrintPos(32, 2);
    display.print(pageTitle);
  } else {
    // Course Direction text
    display.setPrintPos(36, 3);
    display.print(cardinal);
    display.drawLine(56, 0, 56, 14);
    
    // Up arrow
    display.drawBox(31, 6, 3, 5);
    display.drawLine(30, 5, 34, 5); 
    display.drawLine(31, 4, 33, 4); 
    display.drawPixel(32, 3); 

    // Distance
    display.setPrintPos(58, 3);
    display.print(distanceKm, 1);
    display.print("km");
  }
}

void _showTime(uint8_t h, uint8_t m) {
  if (h < 10) {
    display.drawStr(96, 2, "0");
    display.drawStr(102, 2, itoa(h, _internalCharBuf, 10));
  } else {
    display.drawStr(96, 2, itoa(h, _internalCharBuf, 10));
  }
  display.drawStr(108, 1, ":");
  if (m < 10) {
      display.drawStr(114, 2, "0");
      display.drawStr(120, 2, itoa(m, _internalCharBuf, 10));
  } else {
    display.drawStr(114, 2, itoa(m, _internalCharBuf, 10));
  }

}

void _showAccData() {
  display.setPrintPos(0, 16);
  display.print("AccX: ");
  display.print(accEvent.acceleration.x);

  display.setPrintPos(0, 32);
  display.print("AccY: ");
  display.print(accEvent.acceleration.y);

  display.setPrintPos(0, 48);
  display.print("AccZ: ");
  display.print(accEvent.acceleration.z);
}

void _showGyroData() {
  display.setPrintPos(0, 16);
  display.print("X: ");
  display.print(gyroEvent.gyro.x);

  display.setPrintPos(0, 32);
  display.print("Y: ");
  display.print(gyroEvent.gyro.y);

  display.setPrintPos(0, 48);
  display.print("Z: ");
  display.print(gyroEvent.gyro.z);

  display.setPrintPos(64, 16);
  display.print("R: ");
  display.print(gyroEvent.gyro.roll);

  display.setPrintPos(64, 32);
  display.print("P: ");
  display.print(gyroEvent.gyro.pitch);

  display.setPrintPos(64, 48);
  display.print("H: ");
  display.print(gyroEvent.gyro.heading);
}

void _showLocationData() {
  display.setPrintPos(0, 16);
  display.print("lat: ");
  display.print(gps.location.lat(), 6);

  display.setPrintPos(0, 32);
  display.print("lng: ");
  display.print(gps.location.lng(), 6);

  display.setPrintPos(0, 48);
  display.print("altitude: ");
  display.print(gps.altitude.meters());
}

void _showCourseData() {
  display.setPrintPos(0, 16);
  display.print("distance: ");
  display.print(distanceKm);

  display.setPrintPos(0, 32);
  display.print("deg: ");
  display.print(gps.course.deg());

  display.setPrintPos(0, 48);
  display.print("cardinal: ");
  display.println(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "-");
}

void _noSatellite() {
  display.setPrintPos(0, 16);
  display.print("Waiting for satellite");

  uint8_t _dx = millis() / 100;
  uint8_t _dy = _dx % 10;
  if (_dx < 128) {
    display.drawBitmapP(-32 + _dx, 24 + _dy, 4, 32, _satelliteIconL);
  } else {
    display.drawBitmapP(223 - _dx, 24 + _dy, 4, 32, _satelliteIconL);
  }
}

void _showValueSegment(int value) {
  if (value / 100 != 0) {
    drawDigit(5, 16, value / 100);
  }
  drawDigit(5 + 1 * (_sl + 3 * _st), 16, (value / 10) % 10);
  display.drawBox(5 + 2 * (_sl + 3 * _st) - 3, 61, 3, 3);
  drawDigit(5 + 2 * (_sl + 3 * _st), 16, value % 10);

  display.setPrintPos(100, 56);
  display.print("km/h");
}

void _drawHSegment(uint8_t x, uint8_t y) {
  display.drawBox(x, y, _sl - 2, _st);

  display.drawPixel(x - 1, y + 1);
  display.drawPixel(x - 2, y + 2);
  display.drawPixel(x - 1, y + 2);
  display.drawPixel(x - 1, y + 3);

  x += _sl - 2;
  display.drawPixel(x, y + 1);
  display.drawPixel(x + 1, y + 2);
  display.drawPixel(x, y + 2);
  display.drawPixel(x, y + 3);
}

void _drawVSegment(uint8_t x, uint8_t y) {
  display.drawBox(x, y, _st, _sh - 1);

  display.drawPixel(x + 1, y - 1);
  display.drawPixel(x + 2, y - 2);
  display.drawPixel(x + 2, y - 1);
  display.drawPixel(x + 3, y - 1);

  y += _sh - 2;
  display.drawPixel(x + 1, y + 1);
  display.drawPixel(x + 2, y + 2);
  display.drawPixel(x + 2, y + 1);
  display.drawPixel(x + 3, y + 1);
}

void drawDigit(uint8_t x, uint8_t y, uint8_t digit) {
  uint8_t segs = pgm_read_byte(&_digits[digit]);
  if (segs & 0b00000001) _drawHSegment(x + _st + 1, y);
  if (segs & 0b00000010) _drawVSegment(x + _sl + _st, y + _st);
  if (segs & 0b00000100) _drawVSegment(x + _sl + _st, y + _st + _sh + _st);
  if (segs & 0b00001000) _drawHSegment(x + _st + 1, y + 2 * _sh + 2 * _st);
  if (segs & 0b00010000) _drawVSegment(x, y + _st + _sh + _st);
  if (segs & 0b00100000) _drawVSegment(x, y + _st);
  if (segs & 0b01000000) _drawHSegment(x + _st + 1, y + _sh + _st);
}

void _playStartUpTone() {
  analogWrite(SPEAKER_POWER_PIN, 128);
  for (int i = 0; i < sizeof(_melody) / sizeof(_melody[0]); i++) {
    int duration = 1000 / _noteDurations[i];
    tone(SPEAKER_DATA_PIN, _melody[i], duration);
    delay(duration * 1.30);
    noTone(SPEAKER_DATA_PIN);
  }
  noTone(SPEAKER_DATA_PIN);
  analogWrite(SPEAKER_POWER_PIN, 0);
}

void _beep(uint8_t count) {
  analogWrite(SPEAKER_POWER_PIN, 128);
  while (count > 0) {
    tone(SPEAKER_DATA_PIN, 262, 100);
    delay(100 * 1.30);
    count--;
  }
  noTone(SPEAKER_DATA_PIN);
  analogWrite(SPEAKER_POWER_PIN, 0);
}
