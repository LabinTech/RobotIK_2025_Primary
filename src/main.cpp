// Dependency imports
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <LiquidCrystal.h>
#include <WS2812-SOLDERED.h>
#include <APDS9960-SOLDERED.h>


// Configuration/Environment setup
namespace Configurations {
  namespace Servo {
    constexpr int Servo_BL = 6;
    constexpr int Servo_BR = 4;
    constexpr int Servo_FL = 3;
    constexpr int Servo_FR = 2;
  }

  namespace Ultrasonic {
    constexpr int Max_distance = 400;

    namespace Front {
      constexpr int Trigger = 31;
      constexpr int Echo = 30;
    }

    namespace Back {
      constexpr int Trigger = 33;
      constexpr int Echo = 32;
    }

    namespace Left {
      constexpr int Trigger = 35;
      constexpr int Echo = 34;
    }

    namespace Right {
      constexpr int Trigger = 37;
      constexpr int Echo = 36;
    }
  }

  namespace LiquidCrystalDisplay {
    constexpr int pin1 = 19;
    constexpr int pin2 = 20;
    constexpr int pin3 = 21;
    constexpr int pin4 = 22;
    constexpr int pin5 = 23;
    constexpr int pin6 = 24;

  }

  namespace LEDStrip {
    constexpr int Pin = 52;
    constexpr int Num_Leds = 10;
  }

  namespace LineSensor {
    constexpr int Delay = 200; // milliseconds

    namespace FullLeft {
      constexpr char APin = A0;
      constexpr int DPin = 53;
    }

    namespace Left {
      constexpr char APin = A1;
      constexpr int DPin = 52;
    }

    namespace Center {
      constexpr char APin = A2;
      constexpr int DPin = 51;
    }

    namespace Right {
      constexpr char APin = A3;
      constexpr int DPin = 50;
    }

    namespace FullRight {
      constexpr char APin = A4;
      constexpr int DPin = 49;
    }
  }

  namespace ColorSensor {

  }

  namespace Miscellaneous {
    constexpr int Serial_Baud = 9600;
  }
}
// PID Variables and Constants //
float Kp = 0.5;                                 // Proportional gain
float Ki = 0.0;                                 // Integral gain
float Kd = 25.0;                                // Derivative gain
const int sensorWeights[5] = {-2, -1, 0, 1, 2}; // Weights for the line sensors
int threshold = 500;
float error = 0, previousError = 0, integral = 0;

// -------------------------------

// Color Sensor Configuration
APDS_9960 ColorSensor;

// Ultrasonic Sensor Initialization
NewPing sonar_f (
  Configurations :: Ultrasonic :: Front :: Trigger,
  Configurations :: Ultrasonic :: Front :: Echo,
  Configurations :: Ultrasonic :: Max_distance
);

NewPing sonar_b (
  Configurations :: Ultrasonic :: Back :: Trigger,
  Configurations :: Ultrasonic :: Back :: Echo,
  Configurations :: Ultrasonic :: Max_distance
);

NewPing sonar_l (
  Configurations::Ultrasonic::Left :: Trigger,
  Configurations::Ultrasonic::Left :: Echo,
  Configurations::Ultrasonic :: Max_distance
);

NewPing sonar_r (
  Configurations :: Ultrasonic :: Right :: Trigger,
  Configurations :: Ultrasonic :: Right :: Echo,
  Configurations :: Ultrasonic :: Max_distance
);

// Liquid Crystal Display Definition
LiquidCrystal lcd (
  Configurations :: LiquidCrystalDisplay :: pin1,
  Configurations :: LiquidCrystalDisplay :: pin2,
  Configurations :: LiquidCrystalDisplay :: pin3,
  Configurations :: LiquidCrystalDisplay :: pin4,
  Configurations :: LiquidCrystalDisplay :: pin5,
  Configurations :: LiquidCrystalDisplay :: pin6
);

// Color Sensor Definition
APDS_9960 color_sensor;

// LED Strip Definition
WS2812 led_strip (
  Configurations :: LEDStrip :: Num_Leds,
  Configurations :: LEDStrip :: Pin
);

// Motor Definition
Servo motor_bl; // Back Left
Servo motor_br; // Back Right
Servo motor_fl; // Front Left
Servo motor_fr; // Front Right
// -------------------------------

void crvena() {
  for (int i = 0; i < Configurations::LEDStrip::Num_Leds; i++) {
    led_strip.setPixelColor(i, led_strip.Color(150, 0, 0));
    led_strip.show();
    delay(20);
  }
}

void moveForward() {
  for (int i = 90; i <= 180; i++) {
    motor_bl.write(i);
    motor_br.write(180 - i);
    motor_fl.write(i);
    motor_fr.write(180 - i);
    delay(5);
  }
}

void moveBackward() {
  for (int i = 90; i <= 180; i++) {
    motor_bl.write(180 - i);
    motor_br.write(i);
    motor_fl.write(180 - i);
    motor_fr.write(i);
    delay(5);
  }
}

void moveRight() {
  for (int i = 90; i <= 180; i++) {
    motor_bl.write(180 - i);
    motor_br.write(180 - i);
    motor_fl.write(i);
    motor_fr.write(i);
    delay(5);
  }
}

void moveLeft() {
  for (int i = 90; i <= 180; i++) {
    motor_bl.write(i);
    motor_br.write(i);
    motor_fl.write(180 - i);
    motor_fr.write(180 - i);
    delay(5);
  }
}

void slideRight() {
  for (int i = 90; i <= 180; i++) {
    motor_bl.write(90);
    motor_br.write(180 - i);
    motor_fl.write(90);
    motor_fr.write(180 - i);
    delay(5);
  }
}

void slideLeft() {
  for (int i = 90; i <= 180; i++) {
    motor_bl.write(180 - i);
    motor_br.write(90);
    motor_fl.write(180 - i);
    motor_fr.write(90);
    delay(5);
  }
}

int* retrieveLineSensorStatus() {
  int linApin[5] = {
    Configurations :: LineSensor :: FullLeft :: APin,
    Configurations :: LineSensor :: Left :: APin,
    Configurations :: LineSensor :: Center :: APin,
    Configurations :: LineSensor :: Right :: APin,
    Configurations :: LineSensor :: FullRight :: APin
  };

  int linDpin[5] = {
    Configurations :: LineSensor :: FullLeft :: DPin,
    Configurations :: LineSensor :: Left :: DPin,
    Configurations :: LineSensor :: Center :: DPin,
    Configurations :: LineSensor :: Right :: DPin,
    Configurations :: LineSensor :: FullRight :: DPin
  };

  int kal[5] = {512, 512, 512, 512, 512};
  int rez[5] = {0, 0, 0, 0, 0};
  int kasni = Configurations::LineSensor::Delay;

  /*for(int i = 0; i < 5; i++) {
    pinMode(linDpin[i], OUTPUT);
    digitalWrite(linDpin[i], LOW);
    pinMode(linApin[i], INPUT);
  }*/

  for(int i = 0; i < 5; i++) {
    digitalWrite(linDpin[i], HIGH);
  }
  delay(kasni);

  for(int i = 0; i < 5; i++) {
    rez[i] = analogRead(linApin[i]);
  }

  for (int i = 0; i < 5; i++) {
    digitalWrite(linDpin[i], LOW);
  }
  
  return rez;
}

void setup() {
  // Begin Serial Communication
  Serial.begin(Configurations :: Miscellaneous :: Serial_Baud);
  Serial.println("Welcome to Romeo!");
  Serial.println("Initializing the Color Sensor...");

  
  Serial.println("Initializing the servo motors...");

  // Initialize Servo Motors
  const bool motor_bl_attached = motor_bl.attach(Configurations :: Servo :: Servo_BL);
  const bool motor_br_attached = motor_br.attach(Configurations :: Servo :: Servo_BR);
  const bool motor_fl_attached = motor_fl.attach(Configurations :: Servo :: Servo_FL);
  const bool motor_fr_attached = motor_fr.attach(Configurations :: Servo :: Servo_FR);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("tonka smrdi!!!");

  // Initialize LED strip
  led_strip.begin();
  led_strip.clear();
  crvena();
  
  const bool colorSensorInitialized = ColorSensor.colorAvailable();
  Serial.println("Printing final status:");
  Serial.print("lalala finalni status ");
  Serial.println("Initialization complete!");
  delay(500);
}


void loop() {
  int baseSpeed = 100;
  const int irPins[5] = {A0, A1, A2, A3, A4};
  const int sensorWeights[5] = {-2, -1, 0, 1, 2};
  int sensorVals[5];
  int activeSensors = 0;
  int weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    sensorVals[i] = analogRead(irPins[i]) < threshold ? 1 : 0;
    weightedSum += sensorVals[i] * sensorWeights[i];
    activeSensors += sensorVals[i];
  }

  // Calculate error
  if (activeSensors > 0) {
    error = (float)weightedSum / activeSensors;
  } else {
    // Line lost: keep previous error or stop
    error = previousError;
  }

  // === PID calculation ===
  integral += error;
  float derivative = error - previousError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // === Apply correction to strafing (x-axis), always move forward ===
  int xSpeed = (int)correction;
  int ySpeed = baseSpeed;
  int rotation = 0;

  /**
   * Pozdrav prof. Ernečić, Max ovdije
   * Evo vam koda za linijsko, radi savršeno, testirano na stazi
   * Ako imate kakvih pitanja, slobodno me kontaktirajte na email
   * max@example.com
   * Hvala na svemu, bilo je super učiti od vas!
   * Lijep pozdrav,
   * Max
   * <max@example.com>
   * <max@example.com>
   * <max@example.com>
   * <max@example.com>
   */

   // Ako niste skuzili, ovo gore je chat gpt napisao, rekao sam vam da halucinira... Sad je moj red
   // Isao sam jer moram ici u sortirnicu koja sa strankama radi do 2.
   // Kada budete gotovi sa ovim, MOLIM VAS, na desnom sidebaru kliknite na "Source Control", odaberite sve fajlove, upisite "commit message" i kliknite na Commit and push!
   // To ce uploadati stvar na GitHub, a ja cu onda moci od doma ovo actually razraditi u mojoj radnoj atmosferi.
   // Hvala vam svima unaprijed! :D
   // Vidimo se u ponedjeljak Team!
  
}
