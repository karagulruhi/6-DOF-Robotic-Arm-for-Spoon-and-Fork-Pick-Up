

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
uint8_t servonum = 0; 
uint8_t angle = 90;
void loop() {
 
  setServoAngle(servonum ,angle);
  delay(2000);


}
