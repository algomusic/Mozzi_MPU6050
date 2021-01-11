/*  Example reading an MPU6050 accelerometer
    using I2C communication without blocking audio synthesis,
    using Mozzi sonification library.

    Demonstrates use of twi_nonblock functions
    to replace processor-blocking Wire methods.

    This is only a minimum demonstration fragment with no MPU6050 config or error handling.

    Note: The twi_nonblock code is not compatible with Teensy 3.1.

    Circuit: Audio output on digital pin 9.

		Mozzi documentation/API
		https://sensorium.github.io/Mozzi/doc/html/index.html

		Mozzi help/discussion/announcements:
    https://groups.google.com/forum/#!forum/mozzi-users

    Original ADXL345 sample code by Marije Baalman 2012.
    Small modifications by Tim Barrass to retain Mozzi compatibility.
    Converted from ADXL345 to MMA7660 by Roger Cheng
    Converted to MPU6050 by Andrew R. Brown

    This example code is in the public domain.
*/

#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <twi_nonblock.h>

#define CONTROL_RATE 128 // Hz, powers of 2 are most reliable

Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin(SIN2048_DATA);

float gain;

static volatile byte acc_status = 0;
#define ACC_IDLE 0
#define ACC_READING 1
#define ACC_WRITING 2

int accbytedata[14];

#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b
#define MPU6050_ACCEL_OUT_REGISTER    0x3b
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0

void setup_accelero(){
  initialize_twi_nonblock();

  acc_writeTo(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  acc_writeTo(MPU6050_CONFIG_REGISTER, 0x00);
  acc_writeTo(MPU6050_GYRO_CONFIG_REGISTER, 0x00); //0x08
  acc_writeTo(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
  acc_writeTo(MPU6050_PWR_MGMT_1_REGISTER, 0x01);
  
  acc_status = ACC_IDLE;
}

/// ---------- non-blocking version ----------
void initiate_read_accelero(){
  // Reads num bytes starting from address register on device in to _buff array
  // set address of targeted slave
  txAddress = MPU6050_ADDR; //MMA7660_ADDR;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  // put byte in tx buffer
  txBuffer[txBufferIndex] = MPU6050_ACCEL_OUT_REGISTER; // Start reading from register for X
  ++txBufferIndex;
  // update amount in buffer
  txBufferLength = txBufferIndex;

  twi_initiateWriteTo(txAddress, txBuffer, txBufferLength);
  acc_status = ACC_WRITING;
}

void initiate_request_accelero(){
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  byte read = twi_initiateReadFrom(MPU6050_ADDR, 14); // 14 is the number of bytes to read
  acc_status = ACC_READING;
}

void finalise_request_accelero() {
  byte read = twi_readMasterBuffer( rxBuffer, 14 ); 
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  byte i = 0;
  while( rxBufferLength - rxBufferIndex > 0) { // device may send less than requested (abnormal)
    accbytedata[i] = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
    i++;
  }

  acc_status = ACC_IDLE;
}
/// ----end------ non-blocking version ----------

// Writes val to address register on device
void acc_writeTo(byte address, byte val) {
  twowire_beginTransmission(MPU6050_ADDR); // start transmission to device
  twowire_send( address );
  twowire_send( val );
  twowire_endTransmission();
}


void setup(){
  Serial.begin(115200);
  while (!Serial) delay(10); // will pause Zero, Leonardo, etc until serial console opens
  setup_accelero();
  startMozzi(CONTROL_RATE);
  aSin.setFreq(800);
}

int accx;
int accy;
int accz;
float temp;
int gyrox;
int gyroy;
int gyroz;

unsigned long ms = millis();
unsigned long readTime = ms;

void updateControl(){
  ms = millis();
  if (ms > readTime) {
    readTime += 20;
 
    switch( acc_status ){
    case ACC_IDLE:
      accx = (accbytedata[0] << 8 | accbytedata[1]) >> 7; // accelerometer x reading, reduced to 8 bit
      accy = (accbytedata[2] << 8 | accbytedata[3]) >> 7; // accelerometer y reading, 8 bit
      accz = (accbytedata[4] << 8 | accbytedata[5]) >> 7; // accelerometer z reading
      temp = ((accbytedata[6] << 8 | accbytedata[7]) + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;; // temperature reading
      gyrox = (accbytedata[8] << 8 | accbytedata[9]) >> 7; // gyro x reading, reduced to 8 bit
      gyroy = (accbytedata[10] << 8 | accbytedata[11]) >> 7; // gyro y reading, 8 bit
      gyroz = (accbytedata[12] << 8 | accbytedata[13]) >> 7; // gyro z reading
      Serial.print("aX ");Serial.print(accx);
      Serial.print("\taY ");Serial.print(accy);
      Serial.print("\taZ ");Serial.print(accz);
      Serial.print("\tTemp ");Serial.print(temp);
      Serial.print("\tgX ");Serial.print(gyrox);
      Serial.print("\tgY ");Serial.print(gyroy);
      Serial.print("\tgZ ");Serial.print(gyroz);
      Serial.println();
      initiate_read_accelero();
  
      aSin.setFreq(800 + accx * 4);
      gain = 0.5 + accy / 255.0;
      
      break;
    case ACC_WRITING:
      if ( TWI_MTX != twi_state ){
        initiate_request_accelero();
      }
      break;
    case ACC_READING:
      if ( TWI_MRX != twi_state ){
        finalise_request_accelero();
      }
      break;
    }
  }
}


int updateAudio(){
  return aSin.next() * gain; 
}


void loop(){
  audioHook(); // required here
}
