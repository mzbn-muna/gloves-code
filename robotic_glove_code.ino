//Robotic Glove to control the Robotic Arm using Bluetooth

#include<Wire.h>

//Create thumb Sensors
int pinkie = 0; //Pinkie thumb
int finger = 0; //finger thumb
int thumb = 0; //Index thumb

int pinkie_Data = A1;
int finger_Data = A2;
int thumb_Data = A3;

//const int MPU_addr = 0x68;
const int MPU2 = 0x69, MPU1 = 0x68;

//First MPU6050
int16_t AcX1, AcY1, AcZ1, Tmp1, GyX1, GyY1, GyZ1;
int minVal = 265;
int maxVal = 402;
double x;
double y;
double z;

//Second MPU6050
int16_t AcX2, AcY2, AcZ2, Tmp2, GyX2, GyY2, GyZ2;
int minVal2 = 265;
int maxVal2 = 402;
double x2;
double y2;
double z2;

/*Autotune flex parameter
  For Debug Mode. Check the upper and lowe limit of the flex sensors
  3 Flex sensors used. Thumb, Middle, Pinkie
*/
int thumb_high = 0;
int thumb_low = 0;
int finger_high = 0;
int finger_low = 0;
int pinkie_high = 0;
int pinkie_low = 0;

//Stop Caliberating the Flex Sensor when complete
bool bool_caliberate = false;

//How often to send values to the Robotic Arm
int response_time = 100;

void setup() {
  pinMode(3, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);// PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); Wire.begin();
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);// PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(1000);

}
void loop() {
  // Turn ON LED (indicator)
  pinMode(3, HIGH);

  // --- Read angles from MPU1 ---
  GetMpuValue1(MPU1);  // Updates x, y, z from MPU1
  delay(500);          // Small delay to avoid rapid sampling

  // --- Read angles from MPU2 ---
  GetMpuValue2(MPU2);  // Updates x2, y2, z2 from MPU2
  delay(500);

  // --- Always print latest angles from both MPUs ---
  Serial.print("MPU1: X = "); Serial.print(x);
  Serial.print("\tY = "); Serial.print(y);
  Serial.print("\t|| MPU2: X2 = "); Serial.print(x2);
  Serial.print("\tY2 = "); Serial.println(y2);

  // ------------------------------
  // MPU-based gesture commands
  // ------------------------------

  // Move BASE LEFT if MPU1 tilt (x-axis) is between 15° and 55°, y < 30
  if (x > 15 && x < 55 && y < 30) {
    Serial.print("L");
    delay(response_time);
  }

  // Move BASE RIGHT if MPU1 tilt (x-axis) is between 270° and 310°
  if (x < 310 && x > 270) {
    Serial.print("R");
    delay(response_time);
  }

  // Move SHOULDER UP if MPU1 y-axis between 60° and 80°
  if (y > 60 && y < 80) {
    Serial.print("U");
    delay(response_time);
  }

  // Move SHOULDER DOWN if MPU1 y-axis between 270° and 310°
  if (y < 310 && y > 270) {
    Serial.print("D");
    delay(response_time);
  }

  // CLAW Close if MPU2 y2-axis between 50° and 85°
  if (y2 > 50 && y2 < 85) {
    Serial.print("C");
    delay(response_time);
  }

  // CLAW Open if MPU2 y2-axis between 120° and 160°
  if (y2 < 160 && y2 > 120) {
    Serial.print("c");
    delay(response_time);
  }

  // ------------------------------
  // FLEX SENSOR readings
  // ------------------------------

  // Read analog values from each flex sensor
  pinkie = analogRead(pinkie_Data);
  finger = analogRead(finger_Data);
  thumb  = analogRead(thumb_Data);

  // Calibration (runs once after boot)
  if (bool_caliberate == false) {
    delay(1000);  // Let sensors stabilize

    // Set calibration thresholds
    thumb_high  = (thumb  * 1.15);
    thumb_low   = (thumb  * 0.9);
    finger_high = (finger * 1.03);
    finger_low  = (finger * 0.8);
    pinkie_high = (pinkie * 1.06);
    pinkie_low  = (pinkie * 0.8);

    bool_caliberate = true;
  }

  // ------------------------------
  // Flex sensor gesture commands
  // ------------------------------

  // Pinkie bent beyond high threshold → Send "P"
  if (pinkie >= pinkie_high) {
    Serial.print("P");
    delay(response_time);
  }

  // Pinkie bent below low threshold → Send "p"
  if (pinkie <= pinkie_low) {
    Serial.print("p");
    delay(response_time);
  }

  // Thumb bent beyond high threshold → Send "T"
  if (thumb >= thumb_high) {
    Serial.print("T");
    delay(response_time);
  }

  // Thumb bent below low threshold → Send "t"
  if (thumb <= thumb_low) {
    Serial.print("t");
    delay(response_time);
  }

  // Finger bent beyond high threshold → Send "F"
  if (finger >= finger_high) {
    Serial.print("F");
    delay(response_time);
  }

  // Finger bent below low threshold → Send "f"
  if (finger <= finger_low) {
    Serial.print("f");
    delay(response_time);
  }
  else {
    delay(5);  // Small delay when idle
  }
}



void GetMpuValue1(const int MPU) {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers

  AcX1 = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY1 = Wire.read() << 8 |  Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ1 = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Tmp1 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  int xAng = map(AcX1, minVal, maxVal, -90, 90);
  int yAng = map(AcY1, minVal, maxVal, -90, 90);
  int zAng = map(AcZ1, minVal, maxVal, -90, 90);

  GyX1 = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY1 = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ1 = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI) + 4; //offset by 4 degrees to get back to zero
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  //-- Comment to Debug
  Serial.print("AngleX= ");
  Serial.print(x);
  Serial.print("\t");
  //
  Serial.print("AngleY= ");
  Serial.print(y);
  Serial.print("\t");
  
  Serial.print("AngleZ= ");
  Serial.print(z);
  Serial.print("\t");
  Serial.println("-----------------------------------------");


  Serial.print("AcX = ");
  Serial.print(AcX1);
  Serial.print(" | AcY = ");
  Serial.print(AcY1);
  Serial.print(" | AcZ = ");
  Serial.print(AcZ1);
  Serial.print(" | GyX = ");
  Serial.print(GyX1);
  Serial.print(" | GyY = ");
  Serial.print(GyY1);
  Serial.print(" | GyZ = ");
  Serial.println(GyZ1);
}

void GetMpuValue2(const int MPU) {
  delayMicroseconds(1000); // Allow sensor time to refresh
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  AcX2 = Wire.read() << 8 | Wire.read();
  AcY2 = Wire.read() << 8 | Wire.read();
  AcZ2 = Wire.read() << 8 | Wire.read();
  Tmp2 = Wire.read() << 8 | Wire.read();
  GyX2 = Wire.read() << 8 | Wire.read();
  GyY2 = Wire.read() << 8 | Wire.read();
  GyZ2 = Wire.read() << 8 | Wire.read();

  int xAng2 = map(AcX2, minVal2, maxVal2, -90, 90);
  int yAng2 = map(AcY2, minVal2, maxVal2, -90, 90);
  int zAng2 = map(AcZ2, minVal2, maxVal2, -90, 90);

  x2 = RAD_TO_DEG * (atan2(-yAng2, -zAng2) + PI) + 4;
  y2 = RAD_TO_DEG * (atan2(-xAng2, -zAng2) + PI);
  z2 = RAD_TO_DEG * (atan2(-yAng2, -xAng2) + PI);

  Serial.print("AcX2 = "); Serial.print(AcX2);
  Serial.print(" | AcY2 = "); Serial.print(AcY2);
  Serial.print(" | AcZ2 = "); Serial.print(AcZ2);
  Serial.print(" | GyX2 = "); Serial.print(GyX2);
  Serial.print(" | GyY2 = "); Serial.print(GyY2);
  Serial.print(" | GyZ2 = "); Serial.println(GyZ2);

  Serial.print("AngleX2= "); Serial.print(x2);
  Serial.print("\tAngleY2= "); Serial.print(y2);
  Serial.print("\tAngleZ2= "); Serial.println(z2);
}


void debug_flex() {
  //Sends value as a serial monitor to port
  //thumb (Claw open / close)
  Serial.print("Thumb: ");
  Serial.print(thumb);
  Serial.print("\t");
  //  //thumb Params
  Serial.print("thumb High: ");
  Serial.print(thumb_high);
  Serial.print("\t");
  Serial.print("T Low: ");
  Serial.print(thumb_low);
  Serial.print("\t");

  //finger (Claw Further)
  Serial.print("finger: ");
  Serial.print(finger);
  Serial.print("\t");

  //  finger Params
  Serial.print("finger High: ");
  Serial.print(finger_high);
  Serial.print("\t");
  Serial.print("finger Low: ");
  Serial.print(finger_low);
  Serial.print("\t");

  //Pinkie (Claw Further)
  Serial.print("Pinkie: ");

  Serial.print(pinkie);
  Serial.print("\t");

  //  //Pinkie Params
  Serial.print("Pinkie High: ");
  Serial.print(pinkie_high);
  Serial.print("\t");
  Serial.print("Pinkie Low: ");
  Serial.print(pinkie_low);
  Serial.print("\t");
  Serial.println();
}

