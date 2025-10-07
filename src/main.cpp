// Dependency imports
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
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
    constexpr int Address = 0x27;
    constexpr int Columns = 16;
    constexpr int Rows = 2;
  }

  namespace LEDStrip {
    constexpr int Pin = 6;
    constexpr int Num_Leds = 60;
  }

  namespace LineSensor {
    constexpr int Delay = 200; // milliseconds

    namespace FullLeft {
      constexpr int APin = A0;
      constexpr int DPin = 53;
    }

    namespace Left {
      constexpr int APin = A1;
      constexpr int DPin = 52;
    }

    namespace Center {
      constexpr int APin = A2;
      constexpr int DPin = 51;
    }

    namespace Right {
      constexpr int APin = A3;
      constexpr int DPin = 50;
    }

    namespace FullRight {
      constexpr int APin = A4;
      constexpr int DPin = 49;
    }
  }

  namespace ColorSensor {

  }

  namespace Miscellaneous {
    constexpr int Serial_Baud = 9600;
  }
}
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
LiquidCrystal_I2C lcd (
  Configurations :: LiquidCrystalDisplay :: Address,
  Configurations :: LiquidCrystalDisplay :: Columns,
  Configurations :: LiquidCrystalDisplay :: Rows
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
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);

  // Initialize LED strip
  led_strip.begin();
  led_strip.clear();
  
  const bool colorSensorInitialized = ColorSensor.colorAvailable();
  Serial.println("Printing final status:");
  Serial.print("lalala finalni status ");
  Serial.println("Initialization complete!");
  delay(500);
}

void loop() {
  moveForward();
  delay(5000);
  moveBackward();
  delay(5000);
  slideRight();
  delay(5000);
  slideLeft();
  delay(5000);
  return;
}
