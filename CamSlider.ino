
// Copyright Jeff Eberl 2019. License coming, but currently TBD.
// See LICENSE.md

#include <DNSServer.h>
#include <ESPUI.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;

const uint8_t DISTANCE_STEP_PIN    = GPIO_NUM_12;
const uint8_t DISTANCE_DIR_PIN     = GPIO_NUM_26;
const uint8_t DISTANCE_ENABLE_PIN  = GPIO_NUM_13;
const float   STEPS_PER_MM         = 100.0;
const float   MAX_SPEED_MM_S       = 50.0;
const float   ACCELERATION_MM_S_S  = 50.0;
const uint8_t ANGLE_STEP_PIN       = GPIO_NUM_14;
const uint8_t ANGLE_DIR_PIN        = GPIO_NUM_25;
const uint8_t ANGLE_ENABLE_PIN     = GPIO_NUM_13; // This is the same for both?
const float   STEPS_PER_DEG        = 34.87;
const float   MAX_SPEED_DEG_S      = 50.0;
const float   ACCELERATION_DEG_S_S = 50.0;

AccelStepper distanceMotor(AccelStepper::DRIVER, DISTANCE_STEP_PIN, DISTANCE_DIR_PIN);
AccelStepper angleMotor(AccelStepper::DRIVER, ANGLE_STEP_PIN, ANGLE_DIR_PIN);
MultiStepper multiStepper;

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

const char *ssid = "ESPUI";
const char *password = "12345678";

// These should be in units/second
const float MANUAL_ANGLE_RATE = 0.90 * MAX_SPEED_DEG_S;
const float MANUAL_DISTANCE_RATE = 0.90 * MAX_SPEED_MM_S;
const long UPDATE_PERIOD_MILLIS = 1000;

// Global data.
float gTotalTime(60);
float gPreviewFraction(0.0);

float gDesiredAngle(0);
float gDesiredDistance(0);

float gManualAngleRate(0); // Really just a direction
float gManualDistanceRate(0);

float gStartAngle(0);
float gStartDistance(0);

float gEndAngle(20);
float gEndDistance(100);

bool gPlay(false);
bool gFine(false);

long prevTime = 0;
long cycleTime = 0;

void setPreviewFraction(float newFraction) {
  gPreviewFraction = newFraction;
  // set the desired position
  gDesiredAngle = (gPreviewFraction * (gEndAngle - gStartAngle)) + gStartAngle;
  gDesiredDistance = (gPreviewFraction * (gEndDistance - gStartDistance)) + gStartDistance;
}

void changeSeconds(Control sender, int type) {
  gTotalTime = sender.value.toFloat();
  Serial.print(sender.value);
  Serial.println(" Accepted");
}

void slider(Control sender, int type) {
  Serial.print("preview slider set to: ");
  Serial.println(sender.value);
  setPreviewFraction(0.01 * sender.value.toFloat());
}

void reset(Control sender, int type) {
  if (type == B_UP) {
    Serial.println("Reset");
    delay(100);
    ESP.restart();
  }
}

void saveStart(Control sender, int type) {
  if (type == B_UP) {
    Serial.println("Save Start");
    gStartAngle    = gDesiredAngle;
    gStartDistance = gDesiredDistance;
    setPreviewFraction(0.0);
  }
}

void loadStart(Control sender, int type) {
  if (type == B_UP) {
    Serial.println("Load Start");
    gDesiredAngle    = gStartAngle;
    gDesiredDistance = gStartDistance;
    setPreviewFraction(0.0);
  }
}

void saveEnd(Control sender, int type) {
  if (type == B_UP) {
    Serial.println("Save End");
    gEndAngle    = gDesiredAngle;
    gEndDistance = gDesiredDistance;
    setPreviewFraction(1.0);
  }
}

void loadEnd(Control sender, int type) {
  if (type == B_UP) {
    Serial.println("Load End");
    gDesiredAngle    = gEndAngle;
    gDesiredDistance = gEndDistance;
    setPreviewFraction(1.0);
  }
}

void manualControl(Control sender, int value) {
  gPlay = false;
  switch (value) {
    case P_LEFT_DOWN:
      gManualDistanceRate = -1.0;
      Serial.print("Manual Left 1");
      break;
    case P_LEFT_UP:
      gManualDistanceRate = 0.0;
      Serial.print("Manual Left 0");
      break;
    case P_RIGHT_DOWN:
      gManualDistanceRate = +1.0;
      Serial.print("Manual Right 1");
      break;
    case P_RIGHT_UP:
      gManualDistanceRate = 0.0;
      Serial.print("Manual Right 0");
      break;
    case P_FOR_DOWN:
      gManualAngleRate = +1.0;
      Serial.print("Manual Up 1");
      break;
    case P_FOR_UP:
      gManualAngleRate = 0.0;
      Serial.print("Manual Up 0");
      break;
    case P_BACK_DOWN:
      gManualAngleRate = -1.0;
      Serial.print("Manual Down 1");
      break;
    case P_BACK_UP:
      gManualAngleRate = 0.0;
      Serial.print("Manual Down 0");
      break;
  }
  Serial.print(" ");
  Serial.println(sender.id);
}

void playPause(Control sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("Play");
      gPlay = true;
      break;
    case S_INACTIVE:
      Serial.print("Pause");
      gPlay = false;
      break;
  }
  Serial.print(" ");
  Serial.println(sender.id);
}

void fineMotor(Control sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("Fine");
      gFine = true;
      break;
    case S_INACTIVE:
      Serial.print("Gross");
      gFine = false;
      break;
  }
  Serial.print(" ");
  Serial.println(sender.id);
}

void setupWifi() {
  // change the beginning to this if you want to join an existing network
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupUI() {
  ESPUI.pad("Manual Control", false, &manualControl, COLOR_CARROT);
  ESPUI.switcher("Fine", gFine, &fineMotor, COLOR_SUNFLOWER);
  ESPUI.label("Angle:", COLOR_WETASPHALT, String(angleMotor.currentPosition() / STEPS_PER_DEG));
  ESPUI.label("Distance:", COLOR_WETASPHALT, String(distanceMotor.currentPosition() / STEPS_PER_MM));
  ESPUI.label("uptime:", COLOR_WETASPHALT, "0 sec");
  ESPUI.button("Save Start Location", &saveStart, COLOR_EMERALD);
  ESPUI.button("Load Start Location", &loadStart, COLOR_ALIZARIN);
  ESPUI.button("Save End Location", &saveEnd, COLOR_EMERALD);
  ESPUI.button("Load End Location", &loadEnd, COLOR_ALIZARIN);
  ESPUI.number("Total Seconds", &changeSeconds, COLOR_CARROT, gTotalTime, 1, 60 * 60);
  ESPUI.switcher("Play", gPlay, &playPause, COLOR_SUNFLOWER);
  ESPUI.slider("Progress", &slider, COLOR_TURQUOISE, String(gPreviewFraction * 100.0));
  ESPUI.button("RESET", &reset, COLOR_ALIZARIN);

  dnsServer.start(DNS_PORT, "*", apIP);
  ESPUI.begin("ESPUI Control");
}

void setupMotors() {
  distanceMotor.setMaxSpeed(MAX_SPEED_MM_S * STEPS_PER_MM);
  distanceMotor.setAcceleration(ACCELERATION_MM_S_S * STEPS_PER_MM);
  distanceMotor.setSpeed(MAX_SPEED_MM_S * STEPS_PER_MM);
  distanceMotor.setEnablePin(DISTANCE_ENABLE_PIN);
  distanceMotor.setPinsInverted(false, false, true); // dir, step, enable
  distanceMotor.enableOutputs();
  angleMotor.setMaxSpeed(MAX_SPEED_DEG_S * STEPS_PER_DEG);
  angleMotor.setAcceleration(ACCELERATION_DEG_S_S * STEPS_PER_DEG);
  angleMotor.setSpeed(MAX_SPEED_DEG_S * STEPS_PER_DEG);
  // angleMotor.setEnablePin(ANGLE_ENABLE_PIN);
  angleMotor.setPinsInverted(false, false, true);
  angleMotor.enableOutputs();

  multiStepper.addStepper(distanceMotor);
  multiStepper.addStepper(angleMotor);
}

void setup(void) {
  setupWifi();

  setupUI();

  setupMotors();

  cycleTime = millis();
}

void loop(void) {
  dnsServer.processNextRequest();

  float deltaT = 0.001 * static_cast<float>(millis() - cycleTime);
  if (deltaT < 0.0001) {
    delayMicroseconds(100);
    deltaT = 0.001 * static_cast<float>(millis() - cycleTime);
  }
  cycleTime = millis();

  if (gPlay) {
    float now = gPreviewFraction * gTotalTime;
    now += deltaT;
    float newFraction = now / gTotalTime;
    if (newFraction >= 1.0) {
      newFraction = 1.0;
      gPlay = false;
    }
    setPreviewFraction(newFraction);
    long positions[2];
    positions[0] = static_cast<long>(STEPS_PER_MM * gDesiredDistance);
    positions[1] = static_cast<long>(STEPS_PER_DEG * gDesiredAngle);
    multiStepper.moveTo(positions);
  } else {
    float scale(1.0);
    if (gFine) {
      scale = 0.05;
    }

    gDesiredAngle += scale * gManualAngleRate * MANUAL_ANGLE_RATE * deltaT;
    gDesiredDistance += scale * gManualDistanceRate * MANUAL_DISTANCE_RATE * deltaT;
    long positions[2];
    positions[0] = static_cast<long>(STEPS_PER_MM * gDesiredDistance);
    positions[1] = static_cast<long>(STEPS_PER_DEG * gDesiredAngle);
    multiStepper.moveTo(positions);
  }

  // Run the motors
  multiStepper.run();

  if (millis() - prevTime > UPDATE_PERIOD_MILLIS) {
    Serial.print("Distance: ");
    Serial.print(gDesiredDistance);
    Serial.print(" steps: ");
    Serial.print(distanceMotor.currentPosition());
    Serial.print(" speed: ");
    Serial.print(distanceMotor.speed());
    Serial.println();
    ESPUI.print("Distance:", String(distanceMotor.currentPosition() / STEPS_PER_MM) + " mm");
    ESPUI.print("Angle:", String(angleMotor.currentPosition() / STEPS_PER_DEG) + " deg");
    ESPUI.updateSlider("Progress", gPreviewFraction * 100.0);
    ESPUI.updateSwitcher("Play", gPlay);

    ESPUI.print("uptime:", String(static_cast<float>(millis())/1000.0) + " sec"); prevTime = millis();
  }
}
