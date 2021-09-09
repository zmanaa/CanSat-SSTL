#include <Wire.h>
#define BMP 0x77
#define AC1 0xAA
#define AC2 0xAC
#define AC3 0xAE
#define AC4 0xB0
#define AC5 0xB2
#define AC6 0xB4
#define B1 0xB6
#define B2 0xB8
#define MB 0xBA
#define MC 0xBC
#define MD 0xBE
#define pin 3
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
char ID[6];
char UTC[11];
char Lat[10];
char NS[2];
char Long[11];
char EW[2];
char PosFix[2];
char SAT[3];
char Altitude[7];
char allData[40];
char oss = 0 ;
short AC_1, AC_2, AC_3, B_1, B_2, M_B, M_C, M_D ;

short AC_4,AC_5,AC_6 ;


long UT, UP, X1, X2, X3, T, B_3, B_5, B_6, P;

unsigned long B_4 , B_7 ;

short Read_data(int slave , int reg)
{
  short temp ;

    Wire.beginTransmission(slave);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(slave , 2);
    if (Wire.available()==2);
    {
      temp = Wire.read();

      // temp = 0b00000000 , 01010101
      temp = temp<<8;

      // temp = 0b01010101 , 00000000
      
      temp = temp| ( Wire.read() );

      // temp = 0b01010101 , 01101101

      return temp ;
    } 
}

int expect(bool lvl){
  int count=0;
  while(digitalRead(pin)==lvl) count++;
  return count;
  }

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
    while (!Serial); 
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
       // mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
  AC_1 = Read_data(BMP, AC1);
  AC_2 = Read_data(BMP, AC2);
  AC_3 = Read_data(BMP, AC3);
  AC_4 = Read_data(BMP, AC4);
  AC_5 = Read_data(BMP, AC5);
  AC_6 = Read_data(BMP, AC6);
  B_1 = Read_data(BMP, B1);
  B_2 = Read_data(BMP, B1);
  M_B = Read_data(BMP, MB);
  M_C = Read_data(BMP, MC);
  M_D = Read_data(BMP, MD);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  GPSdata();
    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("MPU6050_DMP6 , ");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print(" , ");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(" , ");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal, ");
            Serial.print(aaReal.x);
            Serial.print(" , ");
            Serial.print(aaReal.y);
            Serial.print(" , ");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld , ");
            Serial.print(aaWorld.x);
            Serial.print(" , ");
            Serial.print(aaWorld.y);
            Serial.print(" , ");
            Serial.println(aaWorld.z);

        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose

        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
BMPdata();
DHT();
}
void GPSdata(){
  while(true)
    if (Serial.read()=='$'){
  Serial.readBytesUntil(',',ID,6); 
  ID[5]='\0';// NULL CHARACTER
  if(ID[2]=='G'&& ID[3]=='G'&& ID[4]=='A'){
//    Serial.readBytesUntil('\n',allData,40);
//    Serial.println(allData);
   Serial.readBytesUntil(',',UTC,11); 
   UTC[10]='\0';
    Serial.readBytesUntil(',',Lat,10);
    Lat[9]='\0'; 
    Serial.readBytesUntil(',',NS,2); 
    NS[1]='\0';
    Serial.readBytesUntil(',',Long,11);
    Long[10]='\0'; 
    Serial.readBytesUntil(',',EW,2);
    EW[1]='\0'; 
    Serial.readBytesUntil(',',PosFix,2); 
    PosFix[1]='\0';   
    Serial.readBytesUntil(',',SAT,3);
    SAT[2]='\0'; 
    Serial.readBytesUntil(',',Altitude,3);
    UTC[10]='\0';
    Serial.readBytesUntil(',',Altitude,7);
    Altitude[6]='\0';     
    Serial.print("GPS"); Serial.print(" , ");
    Serial.print(ID);   Serial.print(" , ");
   Serial.print(UTC); Serial.print(" , ");
    Serial.print(Lat); 
    //Serial.print(" , ");
    Serial.print(NS); Serial.print(" , ");
    Serial.print(Long); Serial.print(" , ");
    Serial.print(EW); Serial.print(" , ");
    Serial.print(PosFix);  Serial.print(" , ");
    Serial.print(SAT); Serial.print(" , ");
    Serial.print(Altitude);   Serial.println(" , ");   
    break;
    }
    }
    }
void BMPdata(){
   /// Read uncompensated Temperature

    Wire.beginTransmission(BMP);
    Wire.write(0xf4);
    Wire.write(0x2E);
    Wire.endTransmission(BMP);

    delay(5);  // Can be 5 ms

    UT = Read_data(BMP , 0xF6);

    /// Read uncompensated Pressure

    
    Wire.beginTransmission(BMP);
    Wire.write(0xf4);
    Wire.write( (0x34) | (oss<<6) );
    Wire.endTransmission(BMP);

    delay(5);  // Can be 5 ms


 
    Wire.beginTransmission(BMP);
    Wire.write(0xF6);
    Wire.endTransmission(BMP);
    Wire.requestFrom(BMP , 3);

    if (Wire.available()==3);
    {
      UP = Wire.read();
      // UP = 0b00000000 ,00000000 ,01010101
      
      UP = UP<<8;
      
      // UP = 0b00000000,01010101 ,00000000
      
      
      UP = UP | Wire.read();
    
      // UP = 0b00000000,01010101 , 01101101

      UP = UP<<8;
      
      // UP = 0b01010101 , 01101101 ,00000000
      
      
      UP = UP | Wire.read();
    
      // UP = 0b01010101 , 01101101 , 11111111

      

      UP = UP>>(8-oss);

    }

      //// Calculating Temperature //////

    X1 =   ( (UT - AC_6) * AC_5  ) / pow(2,15);

    X2 = ( M_C * pow(2,11) ) / (X1 + M_D);

    B_5 = X1 + X2 ;

    T = ( B_5+8 ) / pow(2,4);


    ////// Calculating Pressure ////////


    B_6 = B_5 - 4000 ;

    X1 = ( B_2 * ( (B_6 * B_6) / pow(2,12) ) ) / pow(2,11);

    X2 = ( AC_2*B_6/ pow(2,11) );

    X3 = X1+X2;

    B_3 = (((( AC1 * 4 ) + X3 )<<oss) + 2 ) / 4;

    X1 = (AC_3*B_6) / pow(2,13);

    X2 = ( B_1 * ( B_6*B_6 / pow(2,12) ) ) / pow(2,16);

    X3 = ( (X1+X2) + 2 ) /4;

    B_4 = ( AC_4 * (unsigned long)(X3 + 32768) ) / pow(2,15);

    B_7 = ( (unsigned long)UP - B_3 ) * (50000>>oss);


    if (B_7 < 0x80000000) 
    {
      P = (B_7 * 2 ) / B_4;
    }
    else
    {
      P = (B_7/B_4) * 2 ;
    }



    X1 = ( P / pow(2,8) ) * ( P / pow(2,8) );

    X1 = (X1 *3038) / pow(2,16);

    X2 = (-7357 * P ) / pow(2,16);

    P = P +( (X1 + X2 + 3791 ) / pow(2,4) );

    
    
    Serial.print("BMP");Serial.print(" , ");
    Serial.print ( T / 10.0 ); Serial.print(" , ");
    Serial.print ( P/ 100.0 ); Serial.println(" , ");
  }
void DHT()
{
unsigned char RH_int=0,RH_dec=0,T_int=0,T_dec=0,SUM=0;
 unsigned char pulse[80];
pinMode(pin,OUTPUT);
digitalWrite(pin,HIGH);
delay(1000);
digitalWrite(pin,LOW);
delay(18);
digitalWrite(pin,HIGH);
delayMicroseconds(40);
pinMode(pin,INPUT);
if(digitalRead(pin)==LOW)
{while (digitalRead(pin)==LOW){
 //delayMicroseconds(80);
  }
  while (digitalRead(pin)==HIGH){
 //delayMicroseconds(80);
  }
  
  }
  else {Serial.println("NOT RESPONDING");
  return;}
  for ( int i=0; i<80;i++)
    pulse[i]=expect(i%2);
  for(int i=0; i<40;i++){
    unsigned char lowC=pulse[i*2];
    unsigned char HighC=pulse[(i*2)+1];
    if(i<8)
    {RH_int<<=1;
    if (HighC>lowC) RH_int|=1;
      }
    if(i<16)
    {RH_dec<<=1;
    if (HighC>lowC) RH_dec|=1;
      }
    if(i<24)
    {T_int<<=1;
    if (HighC>lowC) T_int|=1;
    }
    if(i<32)
    {T_dec<<=1;
    if (HighC>lowC) T_dec|=1;
    }
    if(i<40)
    {SUM<<=1;
    if (HighC>lowC) SUM|=1;
    }
    }
                if(SUM!=RH_int+RH_dec+T_int+T_dec){
            Serial.println("Error");}
            else 
            {
              Serial.print("DHT , ");
              Serial.print(T_int); Serial.print(".");
              Serial.print(T_dec);Serial.print(" , ");
              Serial.print(RH_int); Serial.print(".");
              Serial.print(RH_dec);
              Serial.println(" , ");
              
              }
}
