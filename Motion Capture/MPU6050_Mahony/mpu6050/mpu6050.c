#include <math.h>
#include "mpu6050.h"
#include "stm32f4xx_it.h"
#include "pca9548.h"

#define delay Delay_ms

Mpu6050_data armR, armL, forearmR, forearmL, legR, legL, upperlegR, upperlegL, head, trunk;

int offsetAcGyA[6] = {314,-57,537133,-30,56,4};
int offsetAcGyB[6] = {-94,-96,537557,-60,-44,-18};
int offsetAcGyC[6] = {-276,-858,536471,40,-26,9};
int offsetAcGyD[6] = {187,-559,537134,-18,-5,-37};
int offsetAcGyE[6] = {-700,-26,536851,-18,9,14};
int offsetAcGyF[6] = {-741,275,537411,-96,19,22};
int offsetAcGyG[6] = {373,-231,536923,-34,25,-10};
int offsetAcGyH[6] = {-393,-269,537034,137,23,-20};
int offsetAcGyI[6] = {471,-1,537306,-23,41,-36};
int offsetAcGyJ[6] = {-320,-92,536680,-2,1,-24};

volatile float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

long sampling_timer;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float axg, ayg, azg, gxrs, gyrs, gzrs;

float SelfTest[6];

float sampleFreq = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval

uint8_t printFlag = 1;

void MPU6050_Init(Mpu6050_data *sensor)
{
  // MPU6050 Initializing & Reset
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00); // set to zero (wakes up the MPU-6050)

  // MPU6050 Clock Type
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01); // Selection Clock 'PLL with X axis gyroscope reference'

  // MPU6050 Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) for DMP
  //writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00); // Default is 1KHz // example 0x04 is 200Hz

  // MPU6050 Gyroscope Configuration Setting
  /* Wire.write(0x00); // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
     Wire.write(0x08); // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
     Wire.write(0x10); // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
     Wire.write(0x18); // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]   */
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18); // FS_SEL=3

  // MPU6050 Accelerometer Configuration Setting
  /* Wire.write(0x00); // AFS_SEL=0, Full Scale Range = +/- 2 [g]
     Wire.write(0x08); // AFS_SEL=1, Full Scale Range = +/- 4 [g]
     Wire.write(0x10); // AFS_SEL=2, Full Scale Range = +/- 8 [g]
     Wire.write(0x18); // AFS_SEL=3, Full Scale Range = +/- 10 [g] */
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x10); // AFS_SEL=2

  // MPU6050 DLPF(Digital Low Pass Filter)
  /*Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
    Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
    Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
    Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
    Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
    Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
    Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz */
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x00); //Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
}


void mpu6050_GetData(Mpu6050_data *sensor)
{
  uint8_t data_org[14]; // original data of accelerometer and gyro
  readBytes(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &data_org[0]);

  AcX = data_org[0] << 8 | data_org[1];  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY = data_org[2] << 8 | data_org[3];  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = data_org[4] << 8 | data_org[5];  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = data_org[6] << 8 | data_org[7];  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = data_org[8] << 8 | data_org[9];  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = data_org[10] << 8 | data_org[11];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = data_org[12] << 8 | data_org[13];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void mpu6050_updateQuaternion(Mpu6050_data *sensor)
{
  axg = (float)(AcX - sensor->Offset[0]) / (float)MPU6050_AXGAIN;
  ayg = (float)(AcY - sensor->Offset[1]) / (float)MPU6050_AYGAIN;
  azg = (float)(AcZ - sensor->Offset[2]) / (float)MPU6050_AZGAIN;
  gxrs = (float)(GyX - sensor->Offset[3]) / (float)MPU6050_GXGAIN * 0.01745329f; //degree to radians
  gyrs = (float)(GyY - sensor->Offset[4]) / (float)MPU6050_GYGAIN * 0.01745329f; //degree to radians
  gzrs = (float)(GyZ - sensor->Offset[5]) / (float)MPU6050_GZGAIN * 0.01745329f; //degree to radians
  // Degree to Radians Pi / 180 = 0.01745329 0.01745329251994329576923690768489
}


void MahonyAHRSupdateIMU(Mpu6050_data *sensor, float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = sensor->q1 * sensor->q3 - sensor->q0 * sensor->q2;
    halfvy = sensor->q0 * sensor->q1 + sensor->q2 * sensor->q3;
    halfvz = sensor->q0 * sensor->q0 - 0.5f + sensor->q3 * sensor->q3;
  
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = sensor->q0;
  qb = sensor->q1;
  qc = sensor->q2;
  sensor->q0 += (-qb * gx - qc * gy - sensor->q3 * gz);
  sensor->q1 += (qa * gx + qc * gz - sensor->q3 * gy);
  sensor->q2 += (qa * gy - qb * gz + sensor->q3 * gx);
  sensor->q3 += (qa * gz + qb * gy - qc * gx); 
  
  // Normalise quaternion
  norm = sqrt(sensor->q0 * sensor->q0 + sensor->q1 * sensor->q1 + sensor->q2 * sensor->q2 + sensor->q3 * sensor->q3);
  sensor->q0 /= norm;
  sensor->q1 /= norm;
  sensor->q2 /= norm;
  sensor->q3 /= norm;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(Mpu6050_data *sensor, float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6];
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(250);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , (((float)selfTest[0] - 1.0f)/30.0f))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , (((float)selfTest[1] - 1.0f)/30.0f))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , (((float)selfTest[2] - 1.0f)/30.0f))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , ((float)selfTest[3] - 1.0f) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , ((float)selfTest[4] - 1.0f) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , ((float)selfTest[5] - 1.0f) ));             // FT[Zg] factory trim calculation
   
 //  Output self-test results and factory trim calculation if desired
 //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
 //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
 //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
 //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0f + 100.0f*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }   
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(Mpu6050_data *sensor, float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);  
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00); 
  delay(200);
  
// Configure device for bias calculation
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_USRL, data[1]);
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_USRH, data[2]);
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_USRL, data[3]);
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, data[4]);
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_USRL, data[5]);

  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_H, data[0]); // might not be supported in MPU6050
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_L_TC, data[1]);
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_H, data[2]);
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_L_TC, data[3]);  
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_H, data[4]);
  writeByte(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_L_TC, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

uint8_t readByte(I2C_HandleTypeDef *i2cx, uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   

  HAL_I2C_Mem_Read(i2cx, address, subAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

  return data; // Return data read from slave register
}

void readBytes(I2C_HandleTypeDef *i2cx, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  HAL_I2C_Mem_Read(i2cx, address, subAddress, I2C_MEMADD_SIZE_8BIT, dest, count, HAL_MAX_DELAY);
}

void writeByte(I2C_HandleTypeDef *i2cx, uint8_t address, uint8_t subAddress, uint8_t data)
{
  HAL_I2C_Mem_Write(i2cx, address, subAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void sensor_init(Mpu6050_data *sensor, I2C_HandleTypeDef *i2cx, uint8_t port, int *offset)
{
	sensor->i2cx = i2cx;
	sensor->port = port;
	sensor->q0 = 1.0f;
	sensor->q1 = 0.0f;
	sensor->q2 = 0.0f;
	sensor->q3 = 0.0f;
	sensor->accelBias[0] = 0;
	sensor->accelBias[1] = 0;
	sensor->accelBias[2] = 0;
	sensor->gyroBias[0] = 0;
	sensor->gyroBias[1] = 0;
	sensor->gyroBias[2] = 0;
	sensor->normalizedQuaternion.w = 1.0;
	sensor->normalizedQuaternion.x = 0.0;
	sensor->normalizedQuaternion.y = 0.0;
	sensor->normalizedQuaternion.z = 0.0;	
	for(uint8_t i = 0; i < 3; i++)
		sensor->Offset[i+3] = offset[i+3];
	selectPort(i2cx, port);	
  // Self Test
  MPU6050SelfTest(sensor, SelfTest);

  // Calibrate MPU6050
//  calibrateMPU6050(sensor, sensor->gyroBias, sensor->accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

  // Initialize MPU6050
  MPU6050_Init(sensor);
}

void sensor_update(Mpu6050_data *sensor)
{
	selectPort(sensor->i2cx, sensor->port);
  // Get raw data
  mpu6050_GetData(sensor);

  // Update raw data to Quaternion form
  mpu6050_updateQuaternion(sensor);
/*
   Initially, sampleFreq should be set as (1000000.0f / (Now - lastUpdate)).
   However, when reading data from multiple sensors, this setting appears to be incorrect.
   In theory, this value should not be dependent on the number of sensors.
   Nevertheless, to ensure the proper functioning of the program, it is necessary to set it as
   (1000000.0f / (number of sensors) / (Now - lastUpdate)).
*/
  Now = micros();
  sampleFreq = (100000.0f / (Now - lastUpdate)); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  //compute data
  MahonyAHRSupdateIMU(sensor, gxrs, gyrs, gzrs, axg, ayg, azg);
	correct_quaternion(sensor);
	//print("%.3f,%.3f,%.3f,%.3f;",sensor->q0,sensor->q1,sensor->q2,sensor->q3);
}

void correct_quaternion(Mpu6050_data *sensor)
{
	Quaternion sensorQuat;
	sensorQuat.w = sensor->q0;
	sensorQuat.x = sensor->q1;
	sensorQuat.y = sensor->q2;
	sensorQuat.z = sensor->q3;
	sensorQuat = quaternion_multiply(sensorQuat, sensor->normalizedQuaternion);	
	if(printFlag == 1)
		print("%.3f,%.3f,%.3f,%.3f;",sensorQuat.w,sensorQuat.x,sensorQuat.y,sensorQuat.z);
}

void init_quaternion(Mpu6050_data *sensor)
{
	Quaternion sensorQuat;
	sensorQuat.w = sensor->q0;
	sensorQuat.x = sensor->q1;
	sensorQuat.y = sensor->q2;
	sensorQuat.z = sensor->q3;
	quaternion_inverseTransformation(&sensorQuat);
	sensor->normalizedQuaternion.w = sensorQuat.w;
	sensor->normalizedQuaternion.x = sensorQuat.x;
	sensor->normalizedQuaternion.y = sensorQuat.y;
	sensor->normalizedQuaternion.z = sensorQuat.z;
}

void sensor_calibrate(Mpu6050_data *sensor)
{
	int16_t AcXt, AcYt, AcZt, GyXt, GyYt, GyZt;
	long Cal_AcX, Cal_AcY, Cal_AcZ, Cal_GyX, Cal_GyY, Cal_GyZ;
	Delay_ms(100); // Wait for I2C to stabilize
	selectPort(sensor->i2cx, sensor->port);
	MPU6050_Init(sensor);

    for (int i = 0; i < 2000; i++) {
      if (i % 200 == 0) print("Calculating .....\r\n");

  uint8_t data_org[14]; // original data of accelerometer and gyro
  readBytes(sensor->i2cx, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &data_org[0]);

  AcXt = data_org[0] << 8 | data_org[1];  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcYt = data_org[2] << 8 | data_org[3];  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZt = (data_org[4] << 8 | data_org[5]) - 4096;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  Tmpt = data_org[6] << 8 | data_org[7];  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyXt = data_org[8] << 8 | data_org[9];  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyYt = data_org[10] << 8 | data_org[11];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZt = data_org[12] << 8 | data_org[13];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      Delay_ms(10);
      Cal_AcX += AcXt;
      Cal_AcY += AcYt;
      Cal_AcZ += AcZt;
      Cal_GyX += GyXt;
      Cal_GyY += GyYt;
      Cal_GyZ += GyZt;
    }

    Cal_AcX /= 2000;
    Cal_AcY /= 2000;
    Cal_AcZ /= 2000;
    Cal_GyX /= 2000;
    Cal_GyY /= 2000;
    Cal_GyZ /= 2000;

    print("End of Calculation\r\n");
    print("AcX = %ld | AcY = %ld | AcZ = %ld | GyX = %ld | GyY = %ld | GyZ = %ld\r\n", Cal_AcX, Cal_AcY, Cal_AcZ, Cal_GyX, Cal_GyY, Cal_GyZ);

}
