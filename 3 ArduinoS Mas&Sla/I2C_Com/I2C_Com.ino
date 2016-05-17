#include <SoftwareSerial.h>
byte bData;				// for incoming serial data(Byte)
int iData;				// for incoming serial data(Int)

bool flag = false;

const int armLength = 12;

const int maxDelay = 3;
int cntDelay;


String leftarm = "left_arm";
String rightarm = "right_arm";

int cntLeftRev;			// for count data(Byte) got from LeftHand.
int cntRightRev;		// for count data(Byte) got from RightHand.

int* iL_Data = new int[14];
int* iR_Data = new int[14];


#define WORK_LEFTHAND
#ifdef WORK_LEFTHAND
	#define rxLPin 8
	#define txLPin 9
	SoftwareSerial leftSerial(rxLPin, txLPin);		//RX, TX
#endif

#define WORK_RIGHTHAND
#ifdef WORK_RIGHTHAND
	#define rxRPin 10
	#define txRPin 11
	SoftwareSerial rightSerial(rxRPin, txRPin);		//RX, TX
#endif

void setup() {
	Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
//  leftSerial.begin(115200);
//  rightSerial.begin(115200);
}

void loop() {
	while( Serial.available() ){
		iData = Serial.read();
	}

	cntLeftRev = 0;
	#ifdef WORK_LEFTHAND
		if(flag){
			// ============================== Left Hand ==============================
			leftSerial.begin(9600);
			//delay(200);
			delay(33);

			Serial.println(leftarm);
			for(int i = 0; i < 12; i++)
				iL_Data[cntLeftRev++] = random(256);

			if(cntLeftRev >= armLength){
			    for(int i = 0; i < sizeof(iL_Data); i++){
			    	Serial.println(iL_Data[i]);
			    }
			}

		    leftSerial.end();
		    //Serial.println();
		    // ============================== Left Hand ============================== End
		    flag = false;
	    }
    #endif
  
	//cntRightRev = 0;
	
    #ifdef WORK_RIGHTHAND
	    // ============================== Right Hand ==============================
	    rightSerial.begin(9600);
	    //delay(200);
	    delay(33);
	    //Serial.print("\t\t\t\t\t\t");

	    while(rightSerial.available() > 1){
	    	bData = rightSerial.read();
	    	iData = (int)bData;
	    	iR_Data[cntRightRev++] = iData;

	    	if( cntRightRev > 13 ){
	    		if( iR_Data[12] == 120 && iR_Data[13] == 13 ){
	    			//Serial.print(rightarm);
	    			Serial.println(rightarm);
	    			for(int i=0;i<12;i++){
		    			//(iR_Data[i]>128) ? Serial.print(iR_Data[i]-256) : Serial.print(iR_Data[i]);
		    			//Serial.print(" ");
		    			//if(i==11)Serial.println();
		    			(iR_Data[i]>128) ? Serial.println(iR_Data[i]-256) : Serial.println(iR_Data[i]);
	    			}

	    			// R: Make random on Lefthand when RightHand got the correct data.
	    			flag = true;
	    		}
	    		
	    		// R: Reset to zerostart over from zero.
	    		cntRightRev = 0;
	    		for(int j = 0; j < sizeof(iR_Data); j++)
	    			iR_Data[j] = 0;

	    		break;
	    	}
	    }

	    rightSerial.end();
	    Serial.println();
	    // ============================== Right Hand ============================== End
    #endif

  //delay 10:33

  //  ascii   DEC   HEX   Char
  //           10     A     LF  : (NL Line feed, new line)
  //           13     D     CR  : (carriage return)
  //           48    30      0
  //           57    39      9
  //           65    41      A
  //           90    5A      Z
  //           97    61      a
  //          122    7A      z
}
