#define F_Length 76.976
#define T_Length 102.460
#define C_Length 45.500
#define PI 3.14159265359

#define NEUTRAL_X 78.994
#define NEUTRAL_Y 78.994
#define NEUTRAL_Z -63.548

#define COXA_FR 1
#define COXA_FL 4
#define COXA_RR 10 
#define COXA_RL 7

#define FEMUR_FR 2
#define FEMUR_FL 5
#define FEMUR_RR 11
#define FEMUR_RL 8

#define TIBIA_FR 3
#define TIBIA_FL 6
#define TIBIA_RR 12
#define TIBIA_RL 9

#include <math.h>
#include <ax12.h>
#include <BioloidController.h>

//
//     [FL]    X    [FR]
//	 #     ^     #
//	   #   |   #
//	     # | #
//	       # - - > Y
//	     #   #
//	   #       #
//	 #           #
//     [RL]         [RR]
//

BioloidController bioloid = BioloidController(1000000);

class Leg;

boolean doIK(Leg * a); //converts end effector goal coordinates to angles.
boolean checkBounds(Leg * a); //checks if angles are within physical bounds.
void writeServos(Leg * a); //writes the current leg object to the corresponding servos.

unsigned long int tA;
unsigned long int tB;

class Leg {
public:
	short int coxaID,
		femurID,
		tibiaID;
	float x, //millimeters
		y,
		z,
		coxa, //radians
		femur,
		tibia;

	short int cFlip,
		fFlip,
		tFlip;

	Leg(short int CID, short int FID, short int TID, short int cf, short int ff, short int tf) {
		coxaID = CID;
		femurID = FID;
		tibiaID = TID;
		cFlip = cf;
		tFlip = tf;
		fFlip = ff;
		x = y = z = 0;
	}

	boolean setGoal(float newX, float newY, float newZ) {
		x = newX + NEUTRAL_X;
		y = newY + NEUTRAL_Y;
		z = newZ + NEUTRAL_Z-20;
		if(!(doIK(this)))
			return false;
		if(!(checkBounds(this)))
			return false;
		writeServos(this);
		return true;
	} 

};
Leg FR (COXA_FR, FEMUR_FR, TIBIA_FR, 1, 1, 1);
Leg FL (COXA_FL, FEMUR_FL, TIBIA_FL, -1, -1, -1);
Leg RL (COXA_RL, FEMUR_RL, TIBIA_RL, -1, 1, 1);
Leg RR (COXA_RR, FEMUR_RR, TIBIA_RR, 1, -1, -1);


void setup() {
	Serial.begin(38400);
	FR.setGoal(0,0,0);
}



void loop() {
	//generate next point
	//pass to FR.setGoal(x,y,z), FL.setGoal() 
	tA=millis()%(2000+6000);
	tB=(millis()+4000)%(2000+6000);
	if (tA <= 2000) {
		FR.setGoal(0,0,10*((sin((2*PI)/2000*tA-PI/2))+1));
		RL.setGoal(0,0,10*((sin((2*PI)/2000*tA-PI/2))+1));
	}
	else if (tA > 2000) {
		FR.setGoal(0,0,0);
		RL.setGoal(0,0,0);
	}
	if (tB <= 2000) {
		FL.setGoal(0,0,10*((sin((2*PI)/2000*tB-PI/2))+1));
		RR.setGoal(0,0,10*((sin((2*PI)/2000*tB-PI/2))+1));
	}
	else if (tB > 2000) {
		FL.setGoal(0,0,0);
		RR.setGoal(0,0,0);
	}
	delay(5);



}

boolean doIK(Leg * a) {
	a->coxa = atan(a->x/a->y);
	float d = sqrt(pow(a->x,2)+pow(a->y,2)) - C_Length;
	a->tibia = acos((float)(pow((float)d,2)+pow((float)a->z,2)-pow((float)F_Length,2)-pow((float)T_Length,2))/(-2*F_Length*T_Length));
	a->femur = asin(T_Length*sin(a->tibia)/(sqrt(pow((float)d,2)+pow((float)a->z,2))))-atan((float)-1*a->z/d);
	a->tibia = 1.37636665-a->tibia;//a->tibia = a->tibia;//-PI/2+a->femur;
	a->femur += 0.196175008;
	a->tibia = 512-a->tFlip*a->tibia*(float)1024/5.23598776;
	a->coxa = 512-a->cFlip*(a->coxa - PI/4)*(float)1024/5.23598776;
	a->femur = 512-a->fFlip*a->femur*(float)1024/5.23598776;
	return true;
}
boolean checkBounds(Leg * a) {
	return true;
}
void writeServos(Leg * a) {
	SetPosition(a->coxaID,(int)a->coxa);
	SetPosition(a->femurID,(int)a->femur);
	SetPosition(a->tibiaID,(int)a->tibia);
}
