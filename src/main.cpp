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
    constexpr int Servo_BL = 9;
    constexpr int Servo_BR = 10;
    constexpr int Servo_FL = 11;
    constexpr int Servo_FR = 12;
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

  namespace WS2812 {
    constexpr int Pin = 6;
    constexpr int Num_Leds = 60;
  }

  namespace LineSensor {

    namespace FullLeft {
      constexpr int APin = A0;
      constexpr int DPin = 45;
    }

    namespace Left {
      constexpr int APin = A1;
      constexpr int DPin = 47;
    }

    namespace Center {
      constexpr int APin = A2;
      constexpr int DPin = 49;
    }

    namespace Right {
      constexpr int APin = A3;
      constexpr int DPin = 51;
    }

    namespace FullRight {
      constexpr int APin = A4;
      constexpr int DPin = 53;
    }
  }

  namespace Miscellaneous {
    constexpr int Serial_Baud = 9600;
  }
}
// -------------------------------

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
  Configurations :: WS2812 :: Num_Leds,
  Configurations :: WS2812 :: Pin
);

// Motor Definition
Servo motor_bl; // Back Left
Servo motor_br; // Back Right
Servo motor_fl; // Front Left
Servo motor_fr; // Front Right
// -------------------------------

void setup() {
  // Begin Serial Communication
  Serial.begin(Configurations :: Miscellaneous :: Serial_Baud);
  Serial.println("Welcome to Božo!");
  Serial.println("Initializing LCD...");

  // Initialize Servo Motors
  motor_bl.attach(Configurations :: Servo :: Servo_BL);
  motor_br.attach(Configurations :: Servo :: Servo_BR);
  motor_fl.attach(Configurations :: Servo :: Servo_FL);
  motor_fr.attach(Configurations :: Servo :: Servo_FR);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);

  // Initialize LED strip
  led_strip.begin();
  led_strip.clear();
}

void loop() {
  delay(1000);
  return;
}
