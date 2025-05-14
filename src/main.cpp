#include <Servo.h>
#include <Arduino.h>

// === PIN-KONFIGURATION ===
#define L_EN 8    
#define R_EN 7   
#define L_PWM 5   
#define R_PWM 6   
#define SERVO_PIN 9

// === SENSOR-PINS ===
#define IR_LEFT A2
#define IR_MIDDLE A3
#define IR_RIGHT A0

// === OBJEKT ===
Servo steeringServo;

// === KONSTANTER ===
const int maxSpeed = 50;
const int minSpeed = 49;
const int servoCenter = 87;

const int stuckThreshold = 20;
const unsigned long stuckTime = 3000;

const int detectThreshold = 100;       // används som "gräns" för när sensorn märker något

const int maxSteer = 30;               // max hur mycket servon får svänga
const int minSteer = 5;                // minsta justering på raksträcka

// === STUCK-DETEKTION ===
int lastIR = 0;
unsigned long lastMoveTime = 0;

void recoverFromStuck() {
    Serial.println("⚠️ Fastnat! Backar...");

    analogWrite(L_PWM, maxSpeed);
    analogWrite(R_PWM, 0);
    steeringServo.write(servoCenter - 25);
    delay(700);

    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
    delay(200);

    steeringServo.write(servoCenter);
    delay(200);
}

void setup() {
    Serial.begin(9600);
    analogReference(DEFAULT);

    steeringServo.attach(SERVO_PIN);
    steeringServo.write(servoCenter);

    // ===STARTMODULE-PINS===
    pinMode(A4, INPUT); // Killpin
    pinMode(A5, INPUT); // Killpin
    pinMode(4, OUTPUT);  // LED på digital pin 4

    pinMode(L_EN, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    digitalWrite(L_EN, HIGH);
    digitalWrite(R_EN, HIGH);

    pinMode(IR_LEFT, INPUT);
    pinMode(IR_MIDDLE, INPUT);
    pinMode(IR_RIGHT, INPUT);

    lastIR = analogRead(IR_MIDDLE);
    lastMoveTime = millis();
}

void loop() {

    if (digitalRead(A4) == HIGH && digitalRead(A5) == HIGH)
  {
    digitalWrite(4, HIGH); // LED på

    int irLeft = analogRead(IR_LEFT);
    int irMiddle = analogRead(IR_MIDDLE);
    int irRight = analogRead(IR_RIGHT);

    Serial.print("Vänster: "); Serial.print(irLeft);
    Serial.print(" | Mitten: "); Serial.print(irMiddle);
    Serial.print(" | Höger: "); Serial.println(irRight);

    // === STUCK ===
    if (abs(irMiddle - lastIR) > stuckThreshold) {
        lastMoveTime = millis();
        lastIR = irMiddle;
    }

    if (millis() - lastMoveTime > stuckTime && irMiddle > detectThreshold) {
        recoverFromStuck();
        lastMoveTime = millis();
        return;
    }

    // === DYNAMISK STYRNING ===
    int angle = servoCenter;
    int diff = abs(irLeft - irRight);

    if (diff > 30) {  // om det verkligen är skillnad mellan sidorna
        int steerAmount = map(diff, 0, 300, minSteer, maxSteer);  // ökar gradvis
        steerAmount = constrain(steerAmount, minSteer, maxSteer);

        if (irLeft > irRight) {
            angle = servoCenter + steerAmount; // hinder vänster → sväng höger
        } else if (irRight > irLeft) {
            angle = servoCenter - steerAmount; // hinder höger → sväng vänster
        }
    }

    steeringServo.write(constrain(angle, 50, 120));

    // === MOTOR KÖR FRAMÅT ===
    analogWrite(R_PWM, maxSpeed);
    analogWrite(L_PWM, 0);
  }
  else{

    digitalWrite(4,LOW); // LED av
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0); // Stanna motorerna
    steeringServo.write(servoCenter); // Styr servon till mitten
  }
    delay(1);
    Serial.print("A4 = ");
    Serial.println(digitalRead(A4));
    Serial.print("A5 = ");
    Serial.println(digitalRead(A5));
}
