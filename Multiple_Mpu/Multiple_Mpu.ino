//  R: Interface: Inter Integrated Circuit(I2C) AND Linux: I2C subsystem (http://afrogue.blogspot.tw/2011/03/interface-inter-integrated-circuiti2c.html
//  R: What's diffrence between I2C Bus and SMBus:  http://eeepage.info/i2c-bus-smbus/http://eeepage.info/i2c-bus-smbus/
// ================================== Mpu_Magnetometers ================================== 
#include <Wire.h>
#include <TimerOne.h>

#define    OUTPUT_READABLE_MAGNETOMETERS
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Initial time
long int ti;
volatile bool intFlag=false;
// ================================== Mpu_Magnetometers ================================== End


// ======================================= PCA9548 ======================================= 
//  R: Reference from: https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
//  R: Reference from: http://www.kerrywong.com/2012/10/08/i2c-multiplexer-shield-testing/comment-page-1/
#define     TCA9548A_ADDRESS            0x70    //X1110 000
#define     MPUCount                    6
int iTCA9548Chn[MPUCount] = {0, 1, 2, 3, 4, 5};
void TcaChnSelect(uint8_t i){
    if (i>7) return;
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(1 << i);
    //Serial.println("endTransmission Before.");    // R: It's Check for sensors work or not, If not then will stuck in Wire.endTrasmission(); .
    Wire.endTransmission();
    //Serial.println("endTransmission After.");
}
// ======================================= PCA9548 ======================================= End


// ================================== Mpu_YawRollPitch =================================== 
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 dmpMpu[MPUCount];

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)      //no need to Array.
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage 

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
// ================================== Mpu_YawRollPitch =================================== End


#define SHOW_MPU_ROLL_PITCH_MAG
//#define TRANSMIT_MPU_ROLL_PITCH_MAG
//  R: To store Roll/Pitch and MagX/MagY from number of sensor(s), 4* numOfSensors = 12;
char str[4 * MPUCount];
int num;
String leftarm  = "left_arm";
String rightarm = "right_arm";
#define CHKLED_PIN 12


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
    while (!Serial);
    // initialize serial communication
    Serial.begin(9600);
    
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);
    pinMode(CHKLED_PIN, OUTPUT);
    
    Serial.println("Initializing MPU9250...");
    Serial.println("If this section stuck, please check pins of I2C(A4/A5 or SDA/SCL) or the SNESOR which has been SELECTING is broken.");
    Serial.println("");
    for( num = 0 ; num < MPUCount ; num++ ){
        digitalWrite(CHKLED_PIN, HIGH);
        Serial.print("\nChannel ");
        Serial.print(iTCA9548Chn[num]);
        Serial.println(" Selecting...");
        TcaChnSelect(iTCA9548Chn[num]);

        // load and configure the DMP
        Serial.print(F(" ======================== Initializing DMP...MpuNum"));
        Serial.print(iTCA9548Chn[num]);
        Serial.println(" ========================");

        devStatus = dmpMpu[num].dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        dmpMpu[num].setXGyroOffset(220);
        dmpMpu[num].setYGyroOffset(76);
        dmpMpu[num].setZGyroOffset(-85);
        dmpMpu[num].setZAccelOffset(1788); // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            dmpMpu[num].setDMPEnabled(true);

            // enable Arduino interrupt detection
            Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
            attachInterrupt(0, dmpDataReady, RISING);
            mpuIntStatus = dmpMpu[num].getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = dmpMpu[num].dmpGetFIFOPacketSize();

            Serial.println(F(" ========================= Initialization Successful ========================= "));

        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));

            Serial.println(F(" ========================== Initialization Failed ========================== "));
            Serial.println(F("Please reset Arduino."));
            while(1);
        }


        #ifdef OUTPUT_READABLE_MAGNETOMETERS
            // ================================== Mpu_Magnetometers ================================== 
            // Set by pass mode for the magnetometers
            I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
          
            // Request continuous magnetometer measurements in 16 bits
            I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

            Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
            Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

            // Store initial time
            ti=millis();
            // ================================== Mpu_Magnetometers ================================== End
        #endif

        Serial.println();
        digitalWrite(CHKLED_PIN, LOW);
        
        delay(1000);
    }
    num = 0;    // R: When program in function of loop(), the variable of num need to initial to zero for select Channel.
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);   
}

// ================================== Mpu_Magnetometers ================================== 
// Counter
long int cpt=0;
void callback()
{ 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}
// ================================== Mpu_Magnetometers ================================== End



void loop() {
    
    // if programming failed, don't try to do anything
    if (!dmpReady) {
        Serial.print("MpuNum");
        Serial.print(iTCA9548Chn[num]);
        Serial.println(F("dmpReady not ready.")); 
        return;
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = dmpMpu[num].getIntStatus();

    // get current FIFO count
    fifoCount = dmpMpu[num].getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        dmpMpu[num].resetFIFO();
        // Serial.print("MpuNum");
        // Serial.print(iTCA9548Chn[num]);
        // Serial.print(F("    FIFO overflow!"));
        // Serial.print(F("    fifoCount= "));
        // Serial.print(fifoCount);
        // Serial.print(F("    mpuIntStatus= "));
        // Serial.print(mpuIntStatus);
        // Serial.print(F("    mpuIntStatus & 0x10= "));
        // Serial.println(mpuIntStatus & 0x10);
        

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = dmpMpu[num].getFIFOCount();

        // read a packet from FIFO
        dmpMpu[num].getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            dmpMpu[num].dmpGetQuaternion(&q, fifoBuffer);
            dmpMpu[num].dmpGetGravity(&gravity, &q);
            dmpMpu[num].dmpGetYawPitchRoll(ypr, &q, &gravity);

            // Serial.print("MpuNum");
            // Serial.print(iTCA9548Chn[num]);

            // Serial.print("    ypr ");
            // Serial.print(ypr[0] * 180/M_PI);     Serial.print(" ");
            // Serial.print(ypr[1] * 180/M_PI);     Serial.print(" ");
            // Serial.print(ypr[2] * 180/M_PI);     Serial.print(" ");

            str[(4 * num) + 0]=(ypr[1] * 180/M_PI);
            str[(4 * num) + 1]=(ypr[2] * 180/M_PI);
            // str[]    y r mx my
            // num=0 -> 0 1  2  3
            // num=1 -> 4 5  6  7
            // num=2 -> 8 9 10 11
            
    #endif

    #ifdef OUTPUT_READABLE_MAGNETOMETERS
            while (!intFlag);
            intFlag=false;
            
            // :::  Magnetometer ::: 
            int16_t mx,my,mz;

            // Read register Status 1 and wait for the DRDY: Data Ready
            uint8_t ST1;
            do{ I2Cread(MAG_ADDRESS,0x02,1,&ST1); }
            while (!(ST1&0x01));
          
            // Read magnetometer data  
            uint8_t Mag[7];
            I2Cread(MAG_ADDRESS,0x03,7,Mag);
            
          
            // Create 16 bits values from 8 bits data
            
            // Magnetometer
            mx=-(Mag[3]<<8 | Mag[2]);
            my=-(Mag[1]<<8 | Mag[0]);
            mz=-(Mag[5]<<8 | Mag[4]);
            
            // Magnetometer
            // Serial.print("\tmag ");
            // Serial.print (mx+200); 
            // Serial.print (" ");
            // Serial.print (my-70);
            // Serial.print (" ");
            // Serial.print (mz-700);  
            // Serial.print ("\t");

            str[(4 * num) + 2]=(mx+100);
            str[(4 * num) + 3]=(my+190);
            // str[]    y r mx my
            // num=0 -> 0 1  2  3
            // num=1 -> 4 5  6  7
            // num=2 -> 8 9 10 11

    #endif

    //R: It's mean MPU overflow yet, we Switch Channel here.
    if( num == MPUCount-1 ) {

        #ifdef SHOW_MPU_ROLL_PITCH_MAG      //  R: FOR READ
            Serial.println(leftarm);
            
            for(int i=0; i<sizeof(str); i++){
//B              if(i%4 == 0){
//                Serial.print("  Mpu");
//                Serial.print(i/4);
//                Serial.print(": ");
//              }
//              Serial.print(" ");
//              Serial.print((int)str[i]);
//              if(i%4 == 0)
//                Serial.print("  ");


              if( i == MPUCount*4 / 2 ){
                Serial.println();
                Serial.println(rightarm);
              }
                
              Serial.print((int)str[i]);
              Serial.println("");
            }//BSerial.println();
        #endif
        #ifdef TRANSMIT_MPU_ROLL_PITCH_MAG  //  R:  FOR TRANSMIT
            for(int i=0; i<sizeof(str); i++){
               //Serial.print((int)str[i]);  //52
               //Serial.print((byte)str[i]); //52
               Serial.write(str[i]);       // 4  'ascii'
               //Serial.print(str[i], HEX);  //34  '34H = 52D = 0011,0100B'
               if( i == sizeof(str)-1 ) {
                Serial.write("x");
                Serial.println();
               }
           }
        #endif
        
        delay(10);
        num = 0;
        
    }else num++;
    TcaChnSelect(iTCA9548Chn[num]);

    // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    
}

