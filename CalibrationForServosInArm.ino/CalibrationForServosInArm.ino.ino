#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PWM sürücüsünü oluştur
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Minimum darbe genişliği (out of 4096)
#define SERVOMAX  600 // Maksimum darbe genişliği (out of 4096)
#define SERVO_FREQ 60  // Analog servolar ~50 Hz'de çalışır



void setup() {
  Serial.begin(9600);
  Serial.println("Servo kontrol test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servolar için frekansı ayarla

  delay(10);
}

void setServoAngle(uint8_t servo, uint8_t angle) {
  int pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Frekansa göre periyot (us)
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12-bit çözünürlük
  Serial.print(pulselength); Serial.println(" us per bit");


  uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulselen);

  // Seri monitöre bilgi yazdırma
  Serial.print("Servo ");
  Serial.print(servo);
  Serial.print(" açısı: ");
  Serial.println(angle);
}
uint8_t servonum1 = 0; 
uint8_t servonum2 = 1; 
uint8_t servonum3 = 2; 
uint8_t servonum4 = 3; 
uint8_t servonum5 = 4; 
uint8_t servonum6 = 6; 

uint8_t angle = 45;
uint8_t angle1 = 90;
void loop() {
 
  setServoAngle(servonum3,angle1);
  delay(1000);


}
