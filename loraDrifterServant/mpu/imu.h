/* This code performs the MPU9250 IMU to predict the position
   The system is runing at 40Hz, the front Y-axis is pointing to West (Yaw = 0 degree)
   MPU library: https://github.com/hideakitai/MPU9250
                Another library: Wire.h
   Port connection: MPU9250 --- ESP32
                      VCC   --- 3V3
                      GND   --- GND
                      SCL   --- 22
                      SDA   --- 21
   Debug use: by send defferent char by serial to esp32, it will output current states:
              "a" :Raw acceleration data, unit: G
              "A" :Processed acceleration data on Earth Frame, unit: m/s2
              "R" :Yaw, Pitch, Roll data(Y axis point to West), unit: degree
              "v" :Return Velocity data, unit: m/s
              "p" :Return Position data, unit: cm
              "m" :Return Magnetometer data, unit: µTesla
              "u" :Increase threshold value for stationary by 0.001
              "d" :Increase threshold value for stationary by 0.001
              "r" :Rest position data to oringe(xyz=0,0,0).
*/
#ifndef IMU_H
#define IMU_H

#define DEBUG_MODE
//#define print_data_serial
#define skip_rot_mat //do not do rotation matrix
//#define without_yaw		//Do not rotate acc with yaw

#include "MPU9250.h"
#include "BasicLinearAlgebra.h"
#include <TinyGPS++.h>
#include <EEPROM.h>

#define ACC_CALI_PARA_ADDR 0
#define MAG_CALI_PARA_ADDR 25

float lms_mb[6] = {1, 0, 1, 0, 1, 0};
bool calibrate_imu = true;

using namespace BLA;
MPU9250 mpu;
#define SAMPLE_PERIOD_ms 50
float T_ = 0.15; //in Sec
uint32_t last_T__=0;
// int Freq_acc = 1000 / SAMPLE_PERIOD_ms; //40Hz
// char in123 = 'a';         // char for input debug
// char incomingByte = 'a';
int count = 0;            //Count for drift velocity elimination

// The TinyGPS++ object
extern TinyGPSPlus gps;

//Define global variables
BLA::Matrix<3> acc = {0.f, 0.f, 0.f};
BLA::Matrix<3> acc_raw = {0.f, 0.f, 0.f};
BLA::Matrix<3> acc_old;
BLA::Matrix<3> Rotation_matrix; 
// float Acc_mag[2] = {0.f, 0.f};
// float Acc[3][2] = {{}, {}};
// float Vel[3][2];
// float Pos[3][2] = {{}, {}};
// float Acc_bias[3] = {0.f, 0.f, 0.f};
float Yaw[2];
// float thread_G = 0.01f;
// int stationary = 1;
// int count_S = 0, count_v = 0;

float Lat_o, Lng_o;

BLA::Matrix<3> U_INS;
// BLA::Matrix<3> Y_INS = {0, 0, 0};
BLA::Matrix<5> X_INS;
// BLA::Matrix<3> N_INS = {0, 0, 0};
BLA::Matrix<3> Y_GPS;
// BLA::Matrix<3> N_GPS = {0, 0, 0};
// BLA::Matrix<3> Bias = {0, 0, 0};
BLA::Matrix<3> Bias_Predic;
// BLA::Matrix<3> U_E = {0, 0, 0};
BLA::Matrix<3> Y_E;
BLA::Matrix<8> X_E_Predic;
BLA::Matrix<8, 8> P;
BLA::Matrix<8, 8, Diagonal<8, float>> P_0;
BLA::Matrix<8, 3> G_k;
BLA::Matrix<3, 8> H;
BLA::Matrix<3, 3, Diagonal<3, float>> R;
BLA::Matrix<3, 3, Diagonal<3, float>> Q;
BLA::Matrix<8, 8> A_E;
BLA::Matrix<8, 3> B_E;
BLA::Matrix<2, 2> C_;
BLA::Matrix<8, 8> big_zero;
BLA::Matrix<8, 8> big_I;
BLA::Matrix<8, 8, Diagonal<8, float>> big_diag;
float BETA = 0.05;

static void Initial_Kalman() {
  //Form matrix reference.
  big_zero.Fill(0);
  big_I.Fill(1);
  big_diag.Fill(1);

  //Setup initial Guess:
  const float P_0_[8] = {10.f, 10.f, 10.f, 10.f, 90.f * PI / 180.f, 5.f, 5.f, 25 * PI / 180.f};
  const float R_[3] = {1.01, 1.01, 1.01 * PI / 180};
  const float Q_[3] = {0.1, 0.1, 0.1 * PI / 180};
  for(int ii = 0; ii < 8; ii++) {
    P_0(ii, ii) = P_0_[ii]*P_0_[ii];
  }
  for(int ii = 0; ii < 3; ii++) {
    R(ii, ii) = R_[ii]*R_[ii];
    Q(ii, ii) = Q_[ii]*Q_[ii];
  }
  Bias_Predic = {0, 0, 0};

  P = P_0;    //Initial The cov of error.
  H.Fill(0);
  //form: H = {0,0,1,1,1,0,0,0, 0,0,1,1,1,0,0,0, 0,0,1,1,1,0,0,0};
  H = (big_zero.Submatrix<3, 2>(1, 1) || big_diag.Submatrix<3, 3>(1, 1) || big_zero.Submatrix<3, 3>(1, 1));
  X_E_Predic.Fill(0);

  //Para matrix for IMU(U) and GPS(Y)
  Y_E.Fill(0);
  G_k.Fill(0);
  X_E_Predic.Fill(0);
  U_INS.Fill(0);
  X_INS.Fill(0);
  Y_GPS.Fill(0);

#ifdef DEBUG_MODE
  Serial << "Initial Kal: \n"
         << "P_0" << P_0 << "\n"
         << "R" << R << "\n"
         << "Q" << Q << "\n"
         << "H" << H << "\n"
         << "X_E_Predic" << X_E_Predic << "\n"

         << "A_E" << A_E << "\n"
         << "B_E" << B_E << "\n";
#endif // DEBUG_MODE
}

void Update_Kalman() {
  uint32_t Current_T_ = millis();
  //T_ = (float)(Current_T_ - last_T__) /1000.f;
  
  //Parameters update
  const BLA::Matrix<3> U_pre = {acc(0) * 9.81f, acc(1) * 9.81f, (Yaw[0]-Yaw[1])/T_};
  C_ = {cos(Yaw[0]), sin(Yaw[0]), -sin(Yaw[0]), cos(Yaw[0])};
  A_E = (big_zero.Submatrix<2, 2>(0, 0) && big_diag.Submatrix<2, 2>(0, 0) && big_zero.Submatrix<4, 2>(0, 0)) ||
        big_zero.Submatrix<8, 3>(0, 0) ||
        (C_ && big_zero.Submatrix<6, 2>(0, 0)) ||
        (big_zero.Submatrix<4, 1>(0, 0) && big_diag.Submatrix<1, 1>(0, 0) && big_zero.Submatrix<3, 1>(0, 0));
  B_E = (C_ && big_zero.Submatrix<6, 2>(0, 0)) ||
        (big_zero.Submatrix<4, 1>(0, 0) && big_diag.Submatrix<1, 1>(0, 0) && big_zero.Submatrix<3, 1>(0, 0));


  //Optimized output X and cov matrix.
  //Update from input
  U_INS = U_pre - Bias_Predic;
  X_INS = {X_INS(0) + T_ * U_INS(0),
           X_INS(1) + T_ * U_INS(1),
           0.5 * T_ * X_INS(0) + X_INS(2),
           0.5 * T_ * X_INS(1) + X_INS(3),
           X_INS(4) + T_ * U_INS(2)
          };
  P = A_E * P * ~A_E + B_E * Q * ~B_E + P_0 * BETA;

  //Update the kalman para
  for(int ii = 0; ii < 3; ii++ ) {
    Y_E(ii) = X_INS(2 + ii) - Y_GPS(ii);
  }
  BLA::Matrix<3, 3> dom_ = H * P * ~H + R;
  BLA::Matrix<3, 3> dom = dom_;
  Invert(dom);
  G_k = P * ~H * dom;
  P = (big_diag - G_k * H) * P;
  BLA::Matrix<8> Bias_0;
  Bias_0.Fill(0);
  for(int ii = 0; ii < 3; ii++) {
    Bias_0(ii + 5) = Bias_Predic(ii);
  }
  X_E_Predic = Bias_0 + G_k * Y_E;

  X_INS -= X_E_Predic.Submatrix<5, 1>(0, 0);
  Bias_Predic = X_E_Predic.Submatrix<3, 1>(5, 0);
  
#ifdef DEBUG_MODE
 Serial << "Time:" << T_  << " last millis: " << (float)last_T__ << " current millis: " << (float)Current_T_
		<< '\n'
		<< "C: " << C_ << '\n'
		<< "Y_E: " << Y_E << "\n"
        << "H: "   << H   << "\n"
		<< "dom_" << dom_ << '\n'
		<< "dom" << dom << '\n'
        << "G_k: " << G_k << "\n"
        << "X_E_Predic: " << X_E_Predic << "\n"
        << "Bias_Predic: " << Bias_Predic << "\n"
        << "U_pre: [" << U_pre(0) << "; " << U_pre(1) << "; " << U_pre(2) << "] "
        << "U_INS: " << U_INS << "\n"
        << "Y_GPS: " << Y_GPS << "\n"
        << "X_INS: " << X_INS << "\n"
        << "P: " <<  P << "\n"
        << "Yaw" << Yaw[0] << " C_" << C_ << "\n"
        << "----------------------------------------------------\n";
#endif // DEBUG_MODE
	last_T__ = Current_T_;
}

void update_ref_location() {
  if(gps.location.isValid()) {
    Lat_o = gps.location.lat();
    Lng_o = gps.location.lng();
  }
  X_INS.Fill(0);
  X_INS(4)=Yaw[0];
}

static const float get_diff_dist(const float oringe, const float update_) {
  return 6372795 * PI / 180 * (update_ - oringe);
}

void get_current_location(float * lat, float * lng) {
  *lat = X_INS(2) * 180 / (6372795 * PI) + Lat_o;
  *lng = X_INS(3) * 180 / (6372795 * PI) + Lng_o;
}

static float get_sqre(BLA::Matrix<3> x, const int n) {
  float sqar = 0.f;
  for(int ii = 0; ii < n; ii++) {
    sqar += x(ii) * x(ii);
  }
  return sqrt(sqar);
}

//Record GPS data
void measure_gps_data() {
  if(gps.location.isValid()) {
    //Get GPS measure in meter from ref point.
    Y_GPS = {get_diff_dist(Lat_o, gps.location.lat()),
             get_diff_dist(Lng_o, gps.location.lng()),
             Yaw[0]
            };
    // Y_GPS = {gps.location.lat(),
    //          gps.location.lng(),
    //          Yaw[0]
    //         };
  }
  else {
    Serial << "Location: INVALID";
  }
#ifdef DEBUG_MODE
  Serial << "Lat: " <<  Y_GPS(0)
         << " Lon: " << Y_GPS(1)
         << " angle: " << Y_GPS(2) << "\n";
#endif // DEBUG_MODE
  //#ifdef DEBUG_MODE
  //  Serial << "Lat: " <<  gps.location.lat()
  //         << " Lon: " << gps.location.lng() << "\n";
  //#endif // DEBUG_MODE
}

//Process Acceleration data to earth frame
void measure_imu_data() {
  acc_raw(0) = lms_mb[0]*mpu.getAccX() + lms_mb[1]; //get from mpu
  acc_raw(1) = lms_mb[2]*mpu.getAccY() + lms_mb[3];
  acc_raw(2) = lms_mb[4]*mpu.getAccZ() + lms_mb[5];
  Rotation_matrix(0) = mpu.getRoll() / 180.f * PI;
  Rotation_matrix(1) = mpu.getPitch() / 180.f * PI;
  Rotation_matrix(2) = mpu.getYaw() / 180.f * PI;

}

void rotate_imu_data() {
  //Set Max Acc
  if(acc_raw(0) >= 2.f) {
    acc_raw(0) = 2.f;
  }
  if(acc_raw(1) >= 2.f) {
    acc_raw(1) = 2.f;
  }
  if(acc_raw(2) >= 2.f) {
    acc_raw(2) = 2.f;
  }

  //rotate from x-axis
  const float acc_y__ = acc_raw(1) * cos(Rotation_matrix(0)) - acc_raw(2) * sin(Rotation_matrix(0));
  const float acc_z__ = acc_raw(2) * cos(Rotation_matrix(0)) + acc_raw(1) * sin(Rotation_matrix(0));
  //rotate from y-axis
  const float acc_x_ = acc_raw(0) * cos(Rotation_matrix(1)) - acc_z__ * sin(Rotation_matrix(1));
  const float acc_z_ = acc_z__ * cos(Rotation_matrix(1)) + acc_raw(0) * sin(Rotation_matrix(1));
  //rotate from z-axis
  acc(0) = acc_x_ * cos(Rotation_matrix(2)) + acc_y__ * sin(Rotation_matrix(2));
  acc(1) = acc_y__ * cos(Rotation_matrix(2)) - acc_x_ * sin(Rotation_matrix(2));
  acc(3) = acc_z_;
#ifdef skip_rot_mat
  acc(0) = acc_raw(0);
  acc(1) = acc_raw(1);
  acc(2) = acc_raw(2);
#endif
#ifdef without_yaw
  acc(0) = acc_x_;
  acc(1) = acc_y__;
  acc(2) = acc_raw(2);
#endif 
  Yaw[1] = Yaw[0];
  Yaw[0] = Rotation_matrix(2);
  // if(calibrate_imu) {
    // for(int ii = 0; ii < 3; ii++) {
      // acc(ii) = lms_mb[2 * ii] * acc(ii) + lms_mb[2 * ii + 1];
    // }
  // }
#ifdef DEBUG_MODE
  Serial << " Acc: " << acc << " Yaw[0]: " << Rotation_matrix(2)
         << " pitch: " << Rotation_matrix(1)
         << " Roll: " << Rotation_matrix(0) << " \n "
		 << " acc_raw: " << acc_raw << " abs: " << float(get_sqre(acc_raw, 3)) << '\n';
#endif // DEBUG_MODE
}

static bool imu_update(){
	int diff=0;
	for(int i=0; i<3; i++){
		if(acc(i) != acc_old(i)){
			diff = 1;
			acc_old(i)=acc(i);
		}
	}
	return diff; 
}

static void read_imu_cali_para(const int address, float * data, const int size) {
  for(int ii = 0; ii < size; ii += sizeof(float)) {
    data[ii / sizeof(float) + address] = EEPROM.readFloat(ii);
    Serial.println(EEPROM.readFloat(ii), 6);
  }
}

#ifdef print_data_serial
static void print_data_name(){
	Serial << "time accx accy accz roll pitch yaw\n";
}

static void print_data(){
	float x = acc(0)*9.81;
	float y = acc(1)*9.81;
	float z = acc(2)*9.81;
	Serial << int(millis()) << " " << x << " " << y << " " << z << " "
			<< Rotation_matrix(0) << " " << Rotation_matrix(1) << " " << Rotation_matrix(2) 
			<< '\n';
}
#endif

void initIMU() {
  Wire.begin();
  delay(1000);
  
  //Yaw[1] = mpu.getYaw() / 180.f * PI;

  //Detect if MPU setup correctly
  if(!mpu.setup(0x68)) {  // change to your own address
    while(1) {
      Serial << "MPU connection failed. Please check your connection with `connection_check` example.\n";
      delay(1000);
    }
  } 
  else {
    Serial << "MPU is connected. \n";
  }

  // MPU calibrate anytime you want to
  Serial << "Accel Gyro calibration will start in 5sec.\n";
  Serial << "Please leave the device still on the flat plane.\n";
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  mpu.setMagBias(-89.91f, 446.60f, -375.83f);  //Set by Observation
  mpu.verbose(false);

  if(calibrate_imu) {
    EEPROM.begin(50);
    read_imu_cali_para(ACC_CALI_PARA_ADDR, lms_mb, sizeof(lms_mb));
    float mag_bias_[3] = {};
    read_imu_cali_para(MAG_CALI_PARA_ADDR, mag_bias_, sizeof(mag_bias_));
    mpu.setMagBias(mag_bias_[0], mag_bias_[1], mag_bias_[2]);  //Set by Observation
  }
  //Initial Kalman and GPS
  Initial_Kalman(); //Initial parameters
  
#ifdef print_data_serial
  //Print data by serial 
  print_data_name();
#endif
  // A sample NMEA stream.
  const char *gpsStream =
    "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
    "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
    "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
    "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
    "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
    "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

  while(*gpsStream) {
    if(gps.encode(*gpsStream++))
      update_ref_location(); //Set first reference location
  }
}

/*-----------------------------Calibration functions---------------------*/
static void LMS_para(const float x[], const int n, float * m_, float * b_) {
  float sumx = 0.f, sumx2 = 0.f, sumxy = 0.f, sumy = 0.f, sumy2 = 0.f;
  float y[n] = {};
  Serial << "Create y array, -1, 0, 1\n";
  //Sum loop
  for(int ii = 0; ii < n; ii++) {
    //Create y array, -1, 0, 1
    if(x[ii] < -0.5f) {
      y[ii] = -1.f;
    }
    else if(x[ii] > 0.5f) {
      y[ii] = 1.f;
    }
    else {
      y[ii] = 0.f;
    }
    Serial << x[ii] << "," << y[ii] << "  ";
    sumx += x[ii];
    sumx2 += x[ii] * x[ii];
    sumxy += x[ii] * y[ii];
    sumy += y[ii];
    sumy2 += y[ii] * y[ii];
  }
  Serial << "Loop done\n";
  const float denom = (n * sumx2 - sumx * sumx);
  if(denom == 0.f) {
    // singular matrix. can't solve the problem.
    Serial << "No solution\n";
    *m_ = 1.f;
    *b_ = 0.f;
  }
  else {
    *m_ = (n * sumxy - sumx * sumy) / denom;
    *b_ = (sumy * sumx2 - sumx * sumxy) / denom;
    Serial << "Solution: m = " << *m_ << " b = " << *b_ << '\n' ;
  }
}

static void write_imu_cali_para(const int address, const float * data, const int size) {
  for(int ii = 0; ii < size; ii += sizeof(float)) {
    EEPROM.writeFloat(ii + address, data[ii / sizeof(float)]);
    //Serial.println(lms_mb[i]);
    Serial.println(EEPROM.readFloat(ii));
  }
  EEPROM.commit();
}

static void calibration_mag() {
  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();
}

static void calibration_imu() {
  int n = 50;
  //  float accx[6 * n] = {};
  //  float accy[6 * n] = {};
  //  float accz[6 * n] = {};
  float acc_cal_temp[3][6 * n] = {{}, {}, {}};
  float m_;
  float b_;
  Serial << " Get 100 samples \n";
  for(int jj = 0; jj < 6; jj++) {
    Serial << "Press k to continue\n";
    while(1) {
      if(Serial.available()) {
        if(Serial.read() == 'k') {
          break;
        }
      }
    }
    for(int ii = jj * 50; ii < (jj * 50 + n); ii++) { //Get n samples of acceleration
      if(mpu.update()) {
        acc_cal_temp[0][ii] = mpu.getAccX();
        acc_cal_temp[1][ii] = mpu.getAccY();
        acc_cal_temp[2][ii] = mpu.getAccZ();
        Serial << "Count: " << ii
               << " x: " << acc_cal_temp[0][ii]
               << " y: " << acc_cal_temp[1][ii]
               << " z: " << acc_cal_temp[2][ii]
               << '\n';
        delay(50);
      }
    }
    Serial << "Flip IMU to next axis\n";
  }

  Serial << "get ACC data done \n";
  //Write into matrix, contains m & b for x, y, z;
  for(int ii = 0; ii < 3; ii++) {
    float acc_temp[6 * n] = {};
    for(int jj = 0; jj < 6 * n; jj++) {
      acc_temp[jj] = acc_cal_temp[ii][jj];
    }
    LMS_para(acc_temp, 6 * n, &lms_mb[ii * 2], &lms_mb[ii * 2 + 1]);
  }
  //Write data into Flash
  //EEPROM.begin(24);
  int count_lms = 0;
  write_imu_cali_para(ACC_CALI_PARA_ADDR, lms_mb, sizeof(lms_mb));
  Serial.println("Store acc para done.");
  Serial << "para:";
  for(int ii = 0; ii < 3; ii++) {
    Serial << " m" << ii + 1 << ": ";
    Serial.print(lms_mb[ii * 2], 5);
    Serial << " b" << ii + 2 << ": ";
    Serial.print(lms_mb[ii * 2 + 1], 5);
  }
  Serial << '\n';
  calibration_mag();
  const float mag_bias_[3] = {mpu.getMagBias(0), mpu.getMagBias(1), mpu.getMagBias(2)};
  write_imu_cali_para(MAG_CALI_PARA_ADDR, mag_bias_, sizeof(mag_bias_));
  Serial.println("Stored all data into flash.");
}

static void print_data(const char print_) {
  if(print_ == 'a' || print_ == 'v' || print_ == 'p' || print_ == 'o' || print_ == 'A' || print_ == 'V' || print_ == 'P') {
    if(print_ == 'a') {
      Serial << "Ax: " << acc(0)
            << " Ay: " << acc(1)
            << " Az_e: " << acc(2) << " ";
      Serial.println(get_sqre(acc, 3), 6);
    }
    else if(print_ == 'v') {
      Serial << "Vx: " << X_INS(0)
            << " Vy: " << X_INS(1)
            //<< " Vz: " << v[2]
            << '\n';
    }
    else if(print_ == 'p') {
      Serial << "Px: " << X_INS(2)
            << " Py: " << X_INS(3)
            //<< " Pz: " << p[2]
            << '\n';
    }
  }
  else {
    return;
  }
}

void read_Serial_input() {
  if(Serial.available()) {
    const char c = Serial.read();
    delay(200);
    switch(c) {
      case 'c':
        Serial << "Run Calibration: ----------------------\n";
        //calibration[0] = 1;
        calibration_imu();
        break;
      case 'r':
        Serial << "Reset position\n";
        for(int ii = 0; ii < 4; ii++) {
          X_INS(ii) = 0;
        }
        break;
      default:
        break;
    }
    print_data(c);
  }
}
  const float error = 0.01f;
  float fix[20] = {};
void check_stable_imu() {
  fix[0] = abs(get_sqre(acc_raw, 3) - 1);
  for(int ii = 1; ii < 20; ii++) {
    memcpy(&fix[ii], &fix[ii - 1], sizeof(float));
  }
  int stationary = 0;
  for(int ii = 0; ii < 15; ii++) { //check last 15
    if(fix[ii] <= error) {		// if less than the error
      stationary++;
    }
  }
  if(stationary >= 15) {
    for(int ii = 0; ii < 2; ii++) {
      acc(ii)=0;
    }
    for(int ii = 0; ii < 20; ii++) {
      fix[ii] = 1.f;
    }
  }
#ifdef DEBUG_MODE
	Serial << "station times: " << stationary
			<< " abs acc: " << fix[0]*1000
			<< " acc: " << acc(0) << ' ' << acc(1)
			<< '\n';
#endif
}



#endif //IMU_H
