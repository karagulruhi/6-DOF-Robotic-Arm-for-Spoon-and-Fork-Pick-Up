#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// PWM sürücüsünü oluştur
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  150 // Minimum darbe genişliği (out of 4096)
#define SERVOMAX  600 // Maksimum darbe genişliği (out of 4096)
#define SERVO_FREQ 60

#define RELAY_PIN 7 
  // Analog servolar ~50 Hz'de çalışır

uint8_t servonum1 = 9;
uint8_t servonum2 =15;
uint8_t servonum3 = 14; 
uint8_t servonum4 = 12; 
uint8_t servonum5 = 11; 
uint8_t servonum6 = 8; 
// uint8_t servonum1 = 6;
// uint8_t servonum2 =14;
// uint8_t servonum3 = 13; 
// uint8_t servonum4 = 11; 
// uint8_t servonum5 = 10; 
// uint8_t servonum6 = 8; 

//gripper

float Pi = 3.14;     // π取值
float L0 = 60 + 15;  // 30为机械臂底部圆盘距离检测边缘距离，根据实际调整,60为圆盘底座固定值。
float L1 = 72;       // 抓取物体表面到第二关节位置高度  unit:mm     72为默认高度
float L2 = 105;      // 第2个关节到第3个关节长度
float L3 = 128;      // 第3关节到第4关节的长度  145
float L4 = 180;      // 第4个关节到手臂尖端的长度（夹具）,包含手爪旋转舵机180

float X_EE, Y_EE, Z_EE;  // 手爪的x轴坐标-左右旋转,y轴坐标-前后伸展,z轴坐标-高度
float Zoffset, D, d, R;
float alpha1, alpha2, alpha3;
float Theta_1, Theta_2, Theta_3, Theta_4;
void Inverse_kinematics(double X_EE, double Y_EE, double Z_EE) {  // 机械臂逆运动学，给定X(左右),Y（前后）,Z（高度）坐标
  D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
  if ((D > L4 + L0 || D < L4 + L0) && Z_EE > (L1 + L2 - L4)) {
    if (X_EE > 0 && Z_EE >= L1) {
      Theta_1 = (atan(Y_EE / X_EE)) * (180.00 / Pi);  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha1 + alpha2);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE > 0 && Z_EE <= L1) {
      Theta_1 = (atan(Y_EE / X_EE)) * (180.00 / Pi);  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha2 - alpha1);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE == 0 && Z_EE >= L1) {
      Theta_1 = 90.00;  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = alpha1 + alpha2;                                                                  //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE == 0 && Z_EE <= L1) {
      Theta_1 = 90.00;  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha2 - alpha1);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE < 0 && Z_EE >= L1) {
      Theta_1 = 90.00 + (90.00 - abs((atan(Y_EE / X_EE)) * (180.00 / Pi)));  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha1 + alpha2);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    } else if (X_EE < 0 && Z_EE <= L1) {
      Theta_1 = 90.00 + (90.00 - abs((atan(Y_EE / X_EE)) * (180.00 / Pi)));  //theta 1
      d = D - L4;
      Zoffset = Z_EE - L1;
      R = sqrt(pow(d, 2) + pow(Zoffset, 2));
      alpha1 = (acos(d / R)) * (180.00 / Pi);
      alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
      Theta_2 = (alpha2 - alpha1);                                                                //theta 2
      Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
      alpha3 = 180.00 - ((180.00 - alpha2 - Theta_3) + alpha1);                                   //alpha 3
      Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
    }
  }
}

bool isStart = false;
void setup() {

  Serial.begin(9600);
  Serial.println("Servo kontrol test!");
  pinMode(RELAY_PIN, OUTPUT); 
  digitalWrite(RELAY_PIN,1);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servolar için frekansı ayarla
  delay(10);
  go_home();
}

// Kameranın Koordinat Sınırları
const int x_camera_min = 0;
const int y_camera_min = 0;
const int x_camera_max = 640;
const int y_camera_max = 480;

const int x_roi_min = 280;
const int y_roi_min = 0;
const int x_roi_max = 560;
const int y_roi_max = 200;
// Robot Kolunun Çalışma Alanı Koordinatları
const float robot_X_min = 30;
const float robot_X_max = 200;
const float robot_Y_min = 30;
const float robot_Y_max = 220;

int currentTheta_1 = 180;
int currentTheta_2 = 120;
int currentTheta_3 = 70;
int currentTheta_4 = 30;
int currentTheta_5 = 90;

int x_robot;
int y_robot;
bool obj_type =-1;
bool pickObj = false;

float mapCameraToRobotX(int y_camera) {
    // Kamera Y'yi ROI y_roi_min/y_roi_max arasına kısıtla
    y_camera = max(y_roi_min, min(y_camera, y_roi_max));
    
    // Oranı hesapla (ters çevirme yapılmıyor, direkt lineer)
    float y_ratio = (float)(y_camera - y_roi_min) / (y_roi_max - y_roi_min);
    
    // Robot X'i hesapla
    return robot_X_min + y_ratio * (robot_X_max - robot_X_min);
}

// Kamera X → Robot Y
float mapCameraToRobotY(int x_camera) {
    // Kamera X'i ROI sınırlarına kısıtla
    x_camera = max(x_roi_min, min(x_camera, x_roi_max));
    
    // Oranı hesapla (Ters çevirme YAPILMADI)
    float x_ratio = (float)(x_camera - x_roi_min) / (x_roi_max - x_roi_min); // Düzeltme burada!
    
    // Robot Y'yi hesapla
    return robot_Y_min + x_ratio * (robot_Y_max - robot_Y_min);
}

void parseData(String data) {
  Serial.print("Alınan veri: ");
  Serial.println(data);
  Serial.println("PROCESSING_START"); 

  int commaIndex1 = data.indexOf(',');
  int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
  int commaIndex3 = data.indexOf(',', commaIndex2 + 1);
  int commaIndex4 = data.indexOf(',', commaIndex3 + 1);

  if (commaIndex1 != -1 && commaIndex2 != -1 && commaIndex3 != -1 && commaIndex4 != -1) {
    float obj_X_min = data.substring(0, commaIndex1).toFloat();
    float obj_Y_min = data.substring(commaIndex1 + 1, commaIndex2).toFloat();
    float obj_X_max = data.substring(commaIndex2 + 1, commaIndex3).toFloat();
    float obj_Y_max = data.substring(commaIndex3 + 1, commaIndex4).toFloat();
    obj_type = (data.substring(commaIndex4 + 1).toInt() == 1); // "1" -> Kaşık, "0" -> Çatal

    // Nesnenin merkez koordinatlarını hesapla
    float obj_corX = (obj_X_min + obj_X_max) / 2.0;
    float obj_corY = (obj_Y_min + obj_Y_max) / 2.0;

 

    // Kameradan gelen koordinatları robot koordinatlarına dönüştür
    x_robot = mapCameraToRobotX(obj_corY);
    y_robot = mapCameraToRobotY(obj_corX);
    x_robot=220;
    y_robot=220;
    Serial.print(" x_robot: ");
    Serial.println(x_robot);
    Serial.print("\y_robot");
    Serial.println(y_robot);
    


    pickObj = false;
    
  } else {
    Serial.println("Hatalı veri formatı! Beklenen format: xmin,ymin,xmax,ymax,object_type");
    pickObj = true;
    Serial.println("READY");
  }
}
void setServoAngle(uint8_t servo, uint8_t angle) {
  uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulselen); // servos[servo][0] servo motorunun pin numarasını belirtir
}

void smoothMove(uint8_t servo, uint8_t startAngle, uint8_t targetAngle, int stepDelay) {
  if (startAngle < targetAngle) {
    for (uint8_t angle = startAngle; angle <= targetAngle; angle++) {
      setServoAngle(servo, angle);
      delay(stepDelay); // Her adım arasında gecikme
    }
  } else {
    for (uint8_t angle = startAngle; angle >= targetAngle; angle--) {
      setServoAngle(servo, angle);
      delay(stepDelay); // Her adım arasında gecikme
    }
  }
}
void moveToPosition(bool act, int x, int y, int z, float tet1 = -1, float tet2 = -1, float tet3 = -1, float tet4 = -1, bool slow = false, int dtime=50) {
  // Ters kinematik hesaplaması
  Inverse_kinematics(x, y, z);

  // Servo motor açılarını kısıtlayarak güncelliyoruz

  // Servo motor açılarını kısıtlayarak güncelliyoruz
  float calculatedTheta_1 = constrain(Theta_1, 0, 180);
  float calculatedTheta_2 = constrain(Theta_2, 0, 172);
  float calculatedTheta_3 = constrain(Theta_3, 0, 178);
  float calculatedTheta_4 = constrain(Theta_4, 0, 170);

  // Kullanıcı tarafından sağlanan açılar varsa, yoksa ters kinematikten gelen açılar kullanılacak
  float targetTheta_1 = (tet1 >= 0) ? constrain(tet1, 0, 180) : calculatedTheta_1;
  float targetTheta_2 = (tet2 >= 0) ? constrain(tet2, 0, 172) : calculatedTheta_2;
  float targetTheta_3 = (tet3 >= 0) ? constrain(tet3, 0, 178) : calculatedTheta_3;
  float targetTheta_4 = (tet4 >= 0) ? constrain(tet4, 0, 170) : calculatedTheta_4;



  if (slow) {
    smoothMove(servonum3, currentTheta_3, 180 - targetTheta_3, dtime);
    currentTheta_3 = 180-targetTheta_3;
    smoothMove(servonum2, currentTheta_2, targetTheta_2, dtime);
    currentTheta_2 = targetTheta_2;
    smoothMove(servonum4, currentTheta_4, targetTheta_4, dtime);
    currentTheta_4 = targetTheta_4;
    smoothMove(servonum1, currentTheta_1, targetTheta_1, dtime);
    currentTheta_1 = targetTheta_1;
    digitalWrite(RELAY_PIN,act);
    
    
  } else {
    setServoAngle(servonum2, targetTheta_2);
    currentTheta_2 = targetTheta_2;
    setServoAngle(servonum1, targetTheta_1);
    currentTheta_1 = targetTheta_1;
    setServoAngle(servonum3, 180 - targetTheta_3);
    currentTheta_3 = 180-targetTheta_3;
    setServoAngle(servonum4, targetTheta_4);
    currentTheta_4 = targetTheta_4;
    digitalWrite(RELAY_PIN,act);
 

  }

  

}





void go_obj() {
  Serial.println("Obje tespit edildi, objeye gidiliyor...");

  Serial.print(x_robot);
  moveToPosition(true, x_robot, y_robot, 200, -1, 120, -1, 10, true, 20);
  moveToPosition(true, x_robot, y_robot, 200, -1, -1, -1, 10, true, 30);
  moveToPosition(true, x_robot, y_robot, 140, -1, -1, -1, 10, true, 200);
  Serial.println("Mıknatıs aktifleştirildi.");

  // 3. Çöp kutusuna taşı
  if (obj_type == 0) { // Çatal
    moveToPosition(true, x_robot, y_robot, 140, 110, 120, 90, 50, true, 30);
    moveToPosition(false, x_robot, y_robot, 140, 110, 50, 110, 50, true, 30);
    
  } else if (obj_type == 1) { // Kaşık
    moveToPosition(true, x_robot, y_robot, 170, 110, 120, 90, 50, true, 30);
    moveToPosition(false, x_robot, y_robot, 170, 110, 70, 80, 50, true, 30);
 
  }
  
  moveToPosition(false, x_robot, y_robot, 140, 110, 120, 90, 50, true, 20);
  go_home();
  Serial.println("READY");
}


void go_home() {
  Serial.println("Robot eve gidiyor...");
  moveToPosition(false, 0, 150,200, 180, -1, -1, -1, true,15);  // Home pozisyonu için sabit açı değerleri
 // Robotun ev pozisyonuna gitmesi için hareket
}
// Kamera Y → Robot X


void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    
    parseData(data);
    
    if (!pickObj) {
     
      go_obj();

    }
  }
  





//   // Debug verileri yazdır
//   #ifdef DEBUG
//   Serial.print(" Servo: ");
//   Serial.print(Theta_1);
//   Serial.print("\t");
//   Serial.print(Theta_2);
//   Serial.print("\t");
//   Serial.print(180 - Theta_3);
//   Serial.print("\t");
//   Serial.println(Theta_4);
//   #endif
}

// Hareket tamamlandığında bu fonksiyon çağrılabilir