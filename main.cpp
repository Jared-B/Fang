#include <math.h>
#include <ax12.h>
#include <BioloidController.h>

typedef signed char uint8;

const uint8 kFrontRight [3] = {1,2,3};
const uint8 kFrontLeft [3] = {4,5,6};
const uint8 kRearLeft [3] = {7,8,9};
const uint8 kRearRight [3] = {10,11,12};
const float kCoxaLength = 45.500;
const float kFemurLength = 76.976;
const float kTibiaLength = 102.460;
const float kNeutralXY = 78.994;
const float kNeutralZ = -63.548; //63
const float kPI = 3.14159265359;
const short int kStepSize = 30;
const short int kStepHeight = 20;
const short int kStepTime = 500; //600
const short int kStepResetTime = 600; //700
const short int kStepTimeOffset [4] = {0,550,0,550};
const float kWalkAngle = 0;
const short int kOffset = 200; //200ms .2s
//
// [FL]    X    [FR]
//	 #     ^     #
//	   #   |   #
//	     # | #
//	       # - - > Y
//	     #   #
//	   #       #
//	 #           #
// [RL]         [RR]
//
class Leg;
BioloidController bioloid = BioloidController(1000000);
boolean doIK(Leg * a);
void walk(Leg a[]);
void writeServos(Leg a[]);
class Leg {
public:
	uint8 coxa_id_, femur_id_, tibia_id_, flip_, x_flip_, y_flip_;
	float x, //millimeters
		y,
		z,
		coxa, //dynamixel positions 0-1024
		femur,
		tibia;

	Leg(const uint8 servo_ids[], uint8 flip, uint8 x_flip, uint8 y_flip) {
		coxa_id_ = servo_ids[0];
		femur_id_ = servo_ids[1];
		tibia_id_ = servo_ids[2];
		x_flip_ = x_flip;
		y_flip_ = y_flip;
		flip_ = flip;
		x = y = z = 0;
	}

	boolean setGoal(float new_x, float new_y, float new_z) {
		x = new_x + kNeutralXY;
		y = new_y + kNeutralXY;
		z = new_z + kNeutralZ-20;
		/*if(!(doIK(this)))
		return false;*/
		doIK(this);
		/*if(!(checkBounds(this)))
		return false;*/
		//writeServos(this);
		return true;
	}
};

Leg legs [4] = {Leg (kFrontRight, 1, 1, 1),
				Leg (kFrontLeft, -1, 1, -1),
				Leg (kRearLeft, 1, -1, -1),
				Leg (kRearRight, -1, -1, 1)};


void setup() {
        //delay(1000);
        //Serial.begin(38400);
        for(int i = 0; i < 3; i++) {
                 ax12SetRegister(254,25,1);
                 delay(1000);
                 ax12SetRegister(254,25,0);
                 delay(1000);
                 
        }
	for(int i = 0; i < 4; i++) {
		legs[i].setGoal(0,0,0);
                writeServos(legs);
	}
        delay(50);
        ax12SetRegister(254,28,64);
        delay(50);
        ax12SetRegister(254,29,64);
        delay(1000);
        //Serial.begin(38400);
}

//int lastMillis = 0;
void loop() {
        //lastMillis=millis();
	walk(legs);
        writeServos(legs);
        delay(30);
        //Serial.println(millis()-lastMillis);        
}

boolean doIK(Leg * a) {
  a->coxa = atan(a->x/a->y);
  float d = sqrt(pow(a->x,2)+pow(a->y,2)) - kCoxaLength;
  a->tibia = acos((float)(pow((float)d,2)+pow((float)a->z,2)-pow((float)kFemurLength,2)-pow((float)kTibiaLength,2))/(-2*kFemurLength*kTibiaLength));
  a->femur = asin(kTibiaLength*sin(a->tibia)/(sqrt(pow((float)d,2)+pow((float)a->z,2))))-atan((float)-1*a->z/d);
  a->tibia = 1.37636665-a->tibia;//a->tibia = a->tibia;//-PI/2+a->femur;
  a->femur += 0.196175008;
  a->tibia = 512-a->flip_*a->tibia*(float)1024/5.23598776;
  a->coxa = 512+a->flip_*(a->coxa - PI/4)*(float)1024/5.23598776;
  a->femur = 512-a->flip_*a->femur*(float)1024/5.23598776;
  return true;
}

void writeServos(Leg a[]) {
	int temp;
        //unsigned int tempDxl;
	int length = 4 + (12 * 3);   // 3 = id + pos(2byte) 12 = number of servos
	int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
	setTXall();
	ax12write(0xFF);
	ax12write(0xFF);
	ax12write(0xFE);
	ax12write(length);
	ax12write(AX_SYNC_WRITE);
	ax12write(AX_GOAL_POSITION_L);
	ax12write(2);
	for (int i = 0; i<4; i++)
	{
		temp = a[i].coxa;
		checksum += (temp & 0xff) + ((temp & 0xff00) >> 8) + a[i].coxa_id_;
		ax12write(a[i].coxa_id_);
		ax12write(temp & 0xff);
		ax12write((temp & 0xff00) >> 8);

		temp = a[i].femur;
		checksum += (temp & 0xff) + ((temp & 0xff00) >> 8) + a[i].femur_id_;
		ax12write(a[i].femur_id_);
		ax12write(temp & 0xff);
		ax12write((temp & 0xff00) >> 8);

		temp = a[i].tibia;
		checksum += (temp & 0xff) + ((temp & 0xff00) >> 8) + a[i].tibia_id_;
		ax12write(a[i].tibia_id_);
		ax12write(temp & 0xff);
		ax12write((temp & 0xff00) >> 8);
	}
	ax12write(0xff - (checksum % 256));
	setRX(0);
}




void walk(Leg a[]) {
	for(int j = 0; j < 4; j++) {
		int time = (millis()+kStepTimeOffset[j])%(kStepTime+kStepResetTime);
		if(time <= kStepTime) {
			a[j].setGoal(a[j].x_flip_*2*kStepSize*cos(kWalkAngle)/2.0*((sin(0.5*(2*kPI)/kStepTime*time-kPI/2))),
				     a[j].y_flip_*2*kStepSize*sin(kWalkAngle)/2.0*((sin(0.5*(2*kPI)/kStepTime*time-kPI/2))),
				     zStep(time));
		}
		else if((time <= kStepTime+kOffset)&&(time>kStepTime))
                    a[j].setGoal(a[j].x_flip_*kStepSize*cos(kWalkAngle)*(1-2.0/(float)kStepResetTime*(time-kStepTime)),
		        a[j].y_flip_*kStepSize*sin(kWalkAngle)*(1-2.0/(float)kStepResetTime*(time-kStepTime)),
  		         zStep(time));
                else if (time>=(kStepTime+kStepResetTime-kOffset))
               		a[j].setGoal(a[j].x_flip_*kStepSize*cos(kWalkAngle)*(1-2.0/(float)kStepResetTime*(time-kStepTime)),
			  	     a[j].y_flip_*kStepSize*sin(kWalkAngle)*(1-2.0/(float)kStepResetTime*(time-kStepTime)),
				     zStep(time-kStepTime-kStepResetTime));   
                 else// if(((time+kOffset)<(kStepTime+kStepResetTime))&&(time>(kStepTime+kOffset)))
                                   a[j].setGoal(a[j].x_flip_*kStepSize*cos(kWalkAngle)*(1-2.0/kStepResetTime*(time-kStepTime)),
				                  a[j].y_flip_*kStepSize*sin(kWalkAngle)*(1-2.0/kStepResetTime*(time-kStepTime)),
				                  -kStepHeight/10.0*cos(kPI/(kStepResetTime-kOffset)*(time-kStepTime-kOffset/2)-kPI/2));
	}
        

}

float zStep (int time){
  return kStepHeight/2.0*((sin((2*kPI)/(kStepTime+2*kOffset)*(time+kOffset)-kPI/2))+1); 
  
  // return kStepHeight/2.0*((sin((2*kPI)/(kStepTime+2*kOffset)*(time+kOffset)-kPI/2))+1);
  //if(z<0);
  //  z=0;
  //return z;
}
