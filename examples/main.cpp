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
    constexpr int Servo_BL = 4;
    constexpr int Servo_BR = 5;
    constexpr int Servo_FL = 2;
    constexpr int Servo_FR = 3;
  }

  namespace Ultrasonic {
    constexpr int Max_distance = 400;

    namespace Front {
      constexpr int Trigger = 22;
      constexpr int Echo = 23;
    }

    namespace Back {
      constexpr int Trigger = 26;
      constexpr int Echo = 27;
    }

    namespace Left {
      constexpr int Trigger = 24;
      constexpr int Echo = 25;
    }

    namespace Right {
      constexpr int Trigger = 28;
      constexpr int Echo = 29;
    }
  }

  namespace LiquidCrystalDisplay {
    constexpr int pin1 = 40;
    constexpr int pin2 = 41;
    constexpr int pin3 = 42;
    constexpr int pin4 = 43;
    constexpr int pin5 = 44;
    constexpr int pin6 = 45;

  }

  namespace LEDStrip {
    constexpr int Pin = 52;
    constexpr int Num_Leds = 10;
  }

  namespace LineSensor {
    constexpr int Delay = 200; // milliseconds
    // Per-sensor thresholds for converting raw analog values to binary ('0'/'1')
    // L1..L8 correspond to indices 0..7
    constexpr int Thresholds[8] = {750, 650, 600, 450, 250, 200, 150, 350};

    namespace L1 {
      constexpr char APin = A0;
    }

    namespace L2 {
      constexpr char APin = A1;
    }

    namespace L3 {
      constexpr char APin = A2;
    }

    namespace L4 {
      constexpr char APin = A3;
    }

    namespace L5 {
      constexpr char APin = A4;
    }

    namespace L6 {
      constexpr char APin = A5;
    }

    namespace L7 {
      constexpr char APin = A6;
    }

    namespace L8 {
      constexpr char APin = A7;
    }
  }

  namespace ColorSensor {

  }

  namespace Miscellaneous {
    constexpr int Serial_Baud = 9600;
  }
}

char lastDir = 'Z'; // S - straight, L - left, R - right
const int maxSpeed = 150;

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
/**
 * @brief Displays a red color on the LED strip.
 */
void showRed() {
  for (int i = 0; i < Configurations :: LEDStrip :: Num_Leds; i++) {
    led_strip.setPixelColor(i, led_strip.Color(150, 0, 0));
    delay(5);
  }
  led_strip.show();
}

/**
 * @brief Displays a yellow color on the LED strip.
 */
void showYellow() {
  for (int i = 0; i < Configurations :: LEDStrip :: Num_Leds; i++) {
    led_strip.setPixelColor(i, led_strip.Color(150, 150, 0));
    delay(5);
  }
  led_strip.show();
}
/**
 * @brief Sets the LED strip to green color.
 */
void showGreen() {
  for (int i = 0; i < Configurations :: LEDStrip :: Num_Leds; i++) {
    led_strip.setPixelColor(i, led_strip.Color(0, 150, 0));
    delay(5);
  }
  led_strip.show();
}

// Movement functions
void stopMoving() {
  motor_bl.write(90);
  motor_br.write(90);
  motor_fl.write(90);
  motor_fr.write(90);
  delay(5);
}
void moveForward() {
  motor_bl.write(maxSpeed);
  motor_br.write(180 - maxSpeed);
  motor_fl.write(maxSpeed);
  motor_fr.write(180 - maxSpeed);
  delay(5);
}

void moveBackward() {
  motor_bl.write(180 - maxSpeed);
  motor_br.write(maxSpeed);
  motor_fl.write(180 - maxSpeed);
  motor_fr.write(maxSpeed);
  delay(5);
}

void moveRight() {
  motor_bl.write(180 - maxSpeed);
  motor_br.write(180 - maxSpeed);
  motor_fl.write(maxSpeed);
  motor_fr.write(maxSpeed);
  delay(5);
}

void moveLeft() {
  motor_bl.write(maxSpeed);
  motor_br.write(maxSpeed);
  motor_fl.write(180 - maxSpeed);
  motor_fr.write(180 - maxSpeed);
  delay(5);
}

void slideRight() {
  motor_bl.write(motor_bl.read() - 30);
  motor_br.write(180 - maxSpeed);
  motor_fl.write(motor_fl.read() - 30);
  motor_fr.write(180 - maxSpeed);
  delay(5);
}

void slideLeft() {
  motor_bl.write(180);
  motor_br.write(motor_br.read() - 30);
  motor_fl.write(180);
  motor_fr.write(motor_fr.read() - 30);
  delay(5);
}

/**
 * @brief Reads raw analog value of a given line sensor (0..1023)
 */
int readLineSensorRaw(int sensorNum) {
  switch (sensorNum) {
    case 0:
      return analogRead(Configurations :: LineSensor :: L1 :: APin);
    case 1:
      return analogRead(Configurations :: LineSensor :: L2 :: APin);
    case 2:
      return analogRead(Configurations :: LineSensor :: L3 :: APin);
    case 3:
      return analogRead(Configurations :: LineSensor :: L4 :: APin);
    case 4:
      return analogRead(Configurations :: LineSensor :: L5 :: APin);
    case 5:
      return analogRead(Configurations :: LineSensor :: L6 :: APin);
    case 6:
      return analogRead(Configurations :: LineSensor :: L7 :: APin);
    case 7:
      return analogRead(Configurations :: LineSensor :: L8 :: APin);
    default:
      return -1; // Invalid sensor number
  }
}

/**
 * @brief Convert raw reading to binary '0'/'1' using configured per-sensor threshold.
 * @param sensorNum Index 0..7 for sensors L1..L8
 * @param raw The raw analog value (0..1023)
 */
char toBitFromRaw(int sensorNum, int raw) {
  // Bounds check: if out of range, default to mid threshold 512
  if (sensorNum < 0 || sensorNum > 7) {
    return (raw > 512) ? '1' : '0';
  }
  const int threshold = Configurations :: LineSensor :: Thresholds[sensorNum];
  return (raw > threshold) ? '1' : '0';
}

void setup() {
  // Begin Serial Communication
  Serial.begin(Configurations :: Miscellaneous :: Serial_Baud);
  Serial.println("Welcome to Romeo!");

  // Initialize LED strip
  Serial.println("Initializing the LED strip...");
  led_strip.begin();
  led_strip.clear();
  //showYellow();
  
  // Initialize Servo Motors
  Serial.println("Initializing the servo motors...");
  const int motor_bl_attached = motor_bl.attach(Configurations :: Servo :: Servo_BL);
  const int motor_br_attached = motor_br.attach(Configurations :: Servo :: Servo_BR);
  const int motor_fl_attached = motor_fl.attach(Configurations :: Servo :: Servo_FL);
  const int motor_fr_attached = motor_fr.attach(Configurations :: Servo :: Servo_FR);

  // Initialize LCD
  Serial.println("Initializing the LCD...");
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  
  const bool colorSensorInitialized = ColorSensor.colorAvailable();
  Serial.println("Printing final status:");
  Serial.print("lalala finalni status ");
  Serial.println("Initialization complete!");
  //showGreen();
  delay(500);
}

void loop() {
  moveForward();
  String lineValue = "";
  for (int i = 0; i < 8; i++) {
    int raw = readLineSensorRaw(i);
    char bit = toBitFromRaw(i, raw);
    // Print raw value for visibility
    Serial.print("Line sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(raw);
    // Build binary pattern string
    lineValue += bit;
  }
  if ((lineValue == "00011000" || lineValue == "00111000" || lineValue == "00011100") && lastDir != 'S') {
    Serial.println("On track");
    //moveForward();
    lastDir = 'S';
  } else if ((lineValue == "01110000" || lineValue == "11100000" || lineValue == "11000000") && lastDir != 'L') {
    Serial.println("Turn left");
    //slideLeft();
    lastDir = 'L';
  } else if ((lineValue == "00001110" || lineValue == "00000111" || lineValue == "000000011") && lastDir != 'R') {
    Serial.println("Turn right");
    //slideRight();
    lastDir = 'R';
  } else if (lineValue == "11111111") {
    Serial.println("No line detected => STOP");
    //stopMoving();
  } else {
    //Serial.println("Continuing like before yayayaya");
    //Serial.println(lineValue);
  }
  delay(1000);
}

/*void loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L:");
  lcd.print(retrieveLineSensorStatus(0));
  lcd.print(" ");
  lcd.print(retrieveLineSensorStatus(1));
  lcd.print(" ");
  lcd.print(retrieveLineSensorStatus(2));

  lcd.setCursor(0, 1);
  lcd.print(retrieveLineSensorStatus(3));
  lcd.print(" ");
  lcd.print(retrieveLineSensorStatus(4));
  delay(250);
  lcd.clear();
  return;
  int values[5] = {0, 0, 0, 0, 0};
  int weights[5] = {-2, -1, 0, 1, 2};

  for (int i = 0; i < 5; i++) {
    values[i] = retrieveLineSensorStatus(i);
  }

  int sum = 0, weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    sum += values[i];
    weightedSum += values[i] * weights[i];
  }
  Serial.println("Sum: ");
  Serial.println(sum);

  if (sum == 0) {
    Serial.println("No line detected => STOP");
    lcd.print("No line");
    stopMoving();
  } else {
    float error = (float)weightedSum / sum;
    
    if (error == 0 && lastDir != 'S') {
      lcd.print("On track");
      moveForward();
      lastDir = 'S';
    } else if (error < 0 && lastDir != 'L') {
      lcd.print("Turn left");
      slideLeft();
      lastDir = 'L';
    } else if (error > 0 && lastDir != 'R') {
      lcd.print("Turn right");
      slideRight();
      lastDir = 'R';
    }
  }
  delay(250);
  lcd.clear();
}
*/