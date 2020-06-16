#include <AiEsp32RotaryEncoder.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ticker.h>
#include <math.h>
#include "driver/ledc.h"
#include <EEPROM.h>
#include <Winnow.h>

#define WHEEL_ENA_PIN 32
#define WHEEL_ENB_PIN 33
#define WHEEL_SW_PIN 25

#define DISP_ENA_PIN 13
#define DISP_ENB_PIN 27
#define DISP_SW_PIN 18

#define MOTOR_PWM_PIN 2
#define MOTOR_DIR_PIN 4

#define PWM_FREQ 10000    // PWM frequency (10k Hz)
#define RESOLUTION 8      // PWM resolution (8 bit, 0-255)

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define REFRESH 1500      // screen refresh delay (ms)

#define DEBOUNCE_TIME 50
#define LONG_PRESS_TIME 3200

#define EEPROM_SIZE 10

void updater();
void memSave();
void memLoad();

String modeString = "Starting";
int16_t displayEncoderDelta = 0;

bool systemStatus = false;
long MAX_PWM = 50;     // maximum PWM value (in percentage)
long THRESH = 5;       // encoder threshold (counts per 'speedCheckInterval')
long ACCEL_TIME = 200; // acceleration time in ms
long DECEL_TIME = 100; // deceleration time in ms
bool REVPOL = false;   // reverse motor polarity

int loc_pwm = 0;
int loc_thresh = 1;
int loc_accel = 2;
int loc_decel = 3;
int loc_revpol = 4;

int16_t wheelSpeed = 0;
int16_t prev_wheelSpeed = 0;
int16_t wheelEncoderDelta = 0;
int wheelDirection = 0;
bool isRunning = false;
double filteredData = 0;

// timer interrupts
Ticker updateDisplayTicker;
Ticker speedCheck;
float updateDisplayInterval = 2000; // OLED screen update rate
float speedCheckInterval = 150;     // encoder calculate velocity interval (ticks/interval)

enum mode_enum
{
  MODE_SETPWM = 1,
  MODE_SETACCEL = 2,
  MODE_SETDECEL = 3,
  MODE_SETTHRESH = 4,
  MODE_REVPOL = 5,
  MODE_SAVE = 6,
  MODE_RUNNING = 7
};

int displayMode = MODE_RUNNING;
int prev_displayMode = -1;

AiEsp32RotaryEncoder wheelEncoder = AiEsp32RotaryEncoder(
    WHEEL_ENA_PIN,
    WHEEL_ENB_PIN,
    WHEEL_SW_PIN,
    -1);

AiEsp32RotaryEncoder displayEncoder = AiEsp32RotaryEncoder(
    DISP_ENA_PIN,
    DISP_ENB_PIN,
    DISP_SW_PIN,
    -1);

struct Button
{
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
  uint8_t brightnessMode;
  uint8_t buttonVal;
  uint8_t buttonLast;
  long btnDnTime;
  long btnUpTime;
  bool ignoreUp;
};
Button display_btn = {DISP_SW_PIN, 0, false, 2, 0, 1, 0, 0, false};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

winnow softFilter(10, 3, 255, 0);

void checkSpeed()
{
  wheelEncoderDelta = wheelEncoder.encoderChanged();

  filteredData = int(softFilter.getFilteredValue(abs(wheelEncoderDelta)));
  // Serial.print(wheelEncoderDelta);
  // Serial.print("  ->  ");
  // Serial.println(filteredData);

  if (wheelEncoderDelta > 0)
  {
    wheelDirection = 1;
  }
  if (wheelEncoderDelta < 0)
  {
    wheelDirection = -1;
  }
  if (wheelEncoderDelta == 0)
  {
    wheelDirection = 0;
  }

  if (systemStatus == true)
  {
    long _targetPWM = map(MAX_PWM, 0, 100, 0, 255);

    if (_targetPWM < 0)
    {
      _targetPWM = 0;
    }
    if (_targetPWM > 255)
    {
      _targetPWM = 255;
    }

    if (wheelDirection == 0 || filteredData < THRESH)
    {
      ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0, DECEL_TIME);
      ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
      delay(10);
      isRunning = false;
    }

    if (wheelDirection == 1 && filteredData > THRESH && isRunning == false)
    {
      if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0) < _targetPWM)
      {
        digitalWrite(MOTOR_DIR_PIN, !REVPOL);
        ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, _targetPWM, ACCEL_TIME);
        ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        delay(10);
        isRunning = true;
      }
    }

    if (wheelDirection == -1 && filteredData > THRESH && isRunning == false)
    {
      if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0) < _targetPWM)
      {
        digitalWrite(MOTOR_DIR_PIN, REVPOL);
        ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, _targetPWM, ACCEL_TIME);
        ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        delay(10);
        isRunning = true;
      }
    }

    // Serial.print("Duty: ");
    // Serial.println(readDuty);

    //   Serial.print(int(abs(wheelEncoderDelta)));
    //   Serial.print("  ");
    //   Serial.println(wheelDirection);
  }
}

void longPressEvent()
{
  Serial.println("Long press.");
  delay(50);

  if (displayMode == MODE_SETPWM)
  {
    Serial.print("PWM: ");
    Serial.print(MAX_PWM);

    long _targetPWM = map(MAX_PWM, 0, 100, 0, 255);

    Serial.print("  -->  ");
    Serial.println(_targetPWM);

    ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, MAX_PWM, ACCEL_TIME);
    ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);

    delay(4000);

    ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0, DECEL_TIME);
    ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);
  }

  if (displayMode == MODE_SAVE)
  {
    Serial.println("Saving data.");

    memSave();

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(10, 10);
    display.println("SAVED.");
    display.display();
    delay(3000);
  }
}

void shortPressEvent()
{
  Serial.print("Mode: ");
  Serial.print(prev_displayMode);
  Serial.print("  -->  ");

  displayMode = displayMode + 1;
  if (displayMode > 7)
  {
    displayMode = 1;
  }

  Serial.println(displayMode);
  delay(25);
}

void displayUpdateValue()
{
  switch (displayMode)
  {
  case MODE_SETPWM:
  {
    MAX_PWM += displayEncoderDelta;

    if (MAX_PWM > 100)
    {
      MAX_PWM = 100;
    }
    if (MAX_PWM < 0)
    {
      MAX_PWM = 0;
    }
    Serial.print("Delta: ");
    Serial.print(displayEncoderDelta);
    Serial.print("   ");
    Serial.print("PWM: ");
    Serial.println(MAX_PWM);
    updater();
  }
  break;
  case MODE_SETACCEL:
  {
    ACCEL_TIME += displayEncoderDelta;

    if (ACCEL_TIME > 250)
    {
      ACCEL_TIME = 250;
    }
    if (ACCEL_TIME < 35)
    {
      ACCEL_TIME = 35;
    }

    Serial.print(displayEncoderDelta);
    Serial.print("  ");
    Serial.println(ACCEL_TIME);
    updater();
  }
  break;
  case MODE_SETDECEL:
  {
    DECEL_TIME += displayEncoderDelta;

    if (DECEL_TIME > 250)
    {
      DECEL_TIME = 250;
    }
    if (DECEL_TIME < 15)
    {
      DECEL_TIME = 15;
    }

    Serial.print(displayEncoderDelta);
    Serial.print("  ");
    Serial.println(DECEL_TIME);
    updater();
  }
  break;
  case MODE_SETTHRESH:
  {
    THRESH += displayEncoderDelta;

    if (THRESH > 150)
    {
      THRESH = 150;
    }
    if (THRESH < 5)
    {
      THRESH = 5;
    }

    Serial.print(displayEncoderDelta);
    Serial.print("  ");
    Serial.println(THRESH);
    updater();
  }
  break;
  case MODE_REVPOL:
  {
    int polarityInt = 0;
    polarityInt += displayEncoderDelta;

    if (polarityInt > 0)
    {
      REVPOL = true;
    }
    if (polarityInt < 0)
    {
      REVPOL = false;
    }
  }
  break;
  case MODE_SAVE:
  {
  }
  break;
  case MODE_RUNNING:
  {
    int statusInt = 0;
    statusInt += displayEncoderDelta;

    if (statusInt > 0)
    {
      systemStatus = true;
    }
    if (statusInt < 0)
    {
      systemStatus = false;
    }
  }
  break;
  default:
    break;
  }
}

void clearDisplay()
{
  display.clearDisplay();
  delay(250);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
}

void updater()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);

  if (displayMode == MODE_SETPWM)
  {
    display.println(modeString);
    display.println(MAX_PWM);
  }
  if (displayMode == MODE_SETACCEL)
  {
    display.println(modeString);
    display.println(ACCEL_TIME);
  }
  if (displayMode == MODE_SETDECEL)
  {
    display.println(modeString);
    display.println(DECEL_TIME);
  }
  if (displayMode == MODE_SETTHRESH)
  {
    display.println(modeString);
    display.println(THRESH);
  }

  if (displayMode == MODE_REVPOL)
  {
    display.println("Reverse motor");
    if (REVPOL == true)
    {
      display.println("True");
    }

    if (REVPOL == false)
    {
      display.println("False");
    }
  }
  if (displayMode == MODE_SAVE)
  {
    display.println("Long press to save");
  }
  if (displayMode == MODE_RUNNING)
  {
    if (systemStatus == true)
    {
      display.println("SYSTEM ON");
    }

    if (systemStatus == false)
    {
      display.println("SYSTEM OFF");
    }

    display.print(wheelDirection);
    display.print("  ");
    display.println(int(filteredData));
  }

  display.display();
  delay(50);
}

void updateDisplayString()
{
  if (displayMode != prev_displayMode)
  {
    switch (displayMode)
    {
    case MODE_SETPWM:
      Serial.println(displayMode);
      modeString = "Max PWM (%): ";
      break;
    case MODE_SETACCEL:
      Serial.println(displayMode);
      modeString = "Set Accel (ms): ";
      break;
    case MODE_SETDECEL:
      Serial.println(displayMode);
      modeString = "Set Decel (ms): ";
      break;
    case MODE_SETTHRESH:
      Serial.println(displayMode);
      modeString = "Speed Threshold: ";
      break;
    case MODE_REVPOL:
      Serial.println(displayMode);
      modeString = "Reverse Pol";
      break;
    case MODE_SAVE:
      Serial.println(displayMode);
      modeString = "Save settings?";
      break;
    case MODE_RUNNING:
      Serial.println(displayMode);
      modeString = "Running...";
      break;
    default:
      break;
    }
    prev_displayMode = displayMode;
  }
}

void setup()
{
  Serial.begin(115200);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed."));
    for (;;)
      ;
  }

  // show splash screen on startup
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10, 10);
  display.println("Starting...");
  display.display();
  delay(3000);

  // load from EEPROM
  EEPROM.begin(EEPROM_SIZE);
  memLoad();

  // pin mode init
  pinMode(display_btn.PIN, INPUT_PULLUP);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, 0);

  // pwm channel init
  ledcSetup(0, PWM_FREQ, RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, 0);
  ledc_fade_func_install(0);

  // encoder init
  wheelEncoder.begin();
  wheelEncoder.setup([] { wheelEncoder.readEncoder_ISR(); });
  wheelEncoder.setBoundaries(-16000, 16000, true);

  displayEncoder.begin();
  displayEncoder.setup([] { displayEncoder.readEncoder_ISR(); });
  displayEncoder.setBoundaries(-16000, 16000, true);

  wheelEncoder.enable();
  displayEncoder.enable();

  // timer interrupt init
  speedCheck.attach_ms(speedCheckInterval, checkSpeed);
  updateDisplayTicker.attach_ms(updateDisplayInterval, updateDisplayString);

  displayMode = MODE_RUNNING;
  updater();
}

void loop()
{
  updateDisplayString();
  updater();
  delay(10);

  displayEncoderDelta = displayEncoder.encoderChanged();

  if (displayEncoderDelta != 0)
  {
    displayUpdateValue();
  }

  // button press handler
  display_btn.buttonVal = digitalRead(display_btn.PIN);
  if (display_btn.buttonVal == LOW && display_btn.buttonLast == HIGH && (millis() - display_btn.btnUpTime) > long(DEBOUNCE_TIME))
  {
    display_btn.btnDnTime = millis();
  }

  if (display_btn.buttonVal == HIGH && display_btn.buttonLast == LOW && (millis() - display_btn.btnDnTime) > long(DEBOUNCE_TIME))
  {
    if (display_btn.ignoreUp == false)
      shortPressEvent();
    else
      display_btn.ignoreUp = false;
    display_btn.btnUpTime = millis();
  }

  if (display_btn.buttonVal == LOW && (millis() - display_btn.btnDnTime) > long(LONG_PRESS_TIME))
  {
    longPressEvent();
    display_btn.ignoreUp = true;
    display_btn.btnDnTime = millis();
  }

  display_btn.buttonLast = display_btn.buttonVal;
}

void memSave()
{
  // this is NOT the correct way to do this -- but I'm tired.
  int accel_norm = ACCEL_TIME / 4;
  int decel_norm = DECEL_TIME / 4;

  EEPROM.write(loc_pwm, MAX_PWM);
  EEPROM.write(loc_thresh, THRESH);
  EEPROM.write(loc_accel, accel_norm);
  EEPROM.write(loc_decel, decel_norm);
  EEPROM.write(loc_revpol, REVPOL);
  EEPROM.commit();

  Serial.println("Saved: ");

  Serial.print("PWM: ");
  Serial.println(MAX_PWM);

  Serial.print("Thresh: ");
  Serial.println(THRESH);

  Serial.print("Accel Time: ");
  Serial.println(ACCEL_TIME);

  Serial.print("Decel Time: ");
  Serial.println(DECEL_TIME);

  Serial.print("Motor Polarity: ");
  Serial.println(REVPOL);
}

void memLoad()
{
  if (EEPROM.read(loc_revpol) > 1)
  {
    // quick check to see if new FW has been loaded (if so, all EEPROM is written to 0xFF)
    // clear, write to default, and skip
    Serial.println("EEPROM IS INVALID. CLEARING.");
    EEPROM.write(loc_pwm, 50);
    EEPROM.write(loc_thresh, 15);
    EEPROM.write(loc_accel, 75);
    EEPROM.write(loc_decel, 50);
    EEPROM.write(loc_revpol, 0);
    EEPROM.commit();

    // now save the default values
    memSave();
  }
  else
  {
    MAX_PWM = EEPROM.read(loc_pwm);
    THRESH = EEPROM.read(loc_thresh);

    int accelVal = EEPROM.read(loc_accel);
    ACCEL_TIME = accelVal * 4;

    int decelVal = EEPROM.read(loc_decel);
    DECEL_TIME = decelVal * 4;

    REVPOL = EEPROM.read(loc_revpol);

    Serial.println("Loaded: ");

    Serial.print("PWM: ");
    Serial.println(MAX_PWM);

    Serial.print("Thresh: ");
    Serial.println(THRESH);

    Serial.print("Accel Time: ");
    Serial.println(ACCEL_TIME);

    Serial.print("Decel Time: ");
    Serial.println(DECEL_TIME);

    Serial.print("Motor Polarity: ");
    Serial.println(REVPOL);
  }
}