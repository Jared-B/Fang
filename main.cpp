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
void gait(float theta, int r,unsigned int tStep,unsigned int tReset,int mz);

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
  Leg FR (COXA_FR, FEMUR_FR, TIBIA_FR, -1, 1, 1);
  Leg FL (COXA_FL, FEMUR_FL, TIBIA_FL, 1, -1, -1);
  Leg RL (COXA_RL, FEMUR_RL, TIBIA_RL, -1, 1, 1);
  Leg RR (COXA_RR, FEMUR_RR, TIBIA_RR, 1, -1, -1);

//unsigned long int loopTime, lastTime;
void setup() {
  Serial.begin(38400);
  FR.setGoal(0,0,0);
  //loopTime = lastTime = 0;
}

int q = 0;

void loop() {
  //gait(theta,r,tStep,tReset,mz)
  //lastTime = millis();
  gait(0,40,250,270,20);
  //loopTime = millis() - lastTime;
  //Serial.println(loopTime);
  q++;
  if(q == 1) {
    for(int j = 0; j < 12; j++) {
      Serial.print(j+1);
      Serial.print(": ");
      Serial.print(ax12GetRegister(j+1, 43, 1));
      Serial.write(10);
    }
  }
  else if (q > 20)
    q = 0;


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
  if ( ((512*(1+a->cFlip)-a->cFlip*a->coxa) <720) && 
       ((512*(1+a->cFlip)-a->cFlip*a->coxa) >300) && 
       ((512*(1+a->fFlip)-a->fFlip*a->femur)<730) && 
       ((512*(1+a->fFlip)-a->fFlip*a->femur)>512)  && 
       ((512*(1+a->tFlip)-a->tFlip*a->tibia)<799) && 
       ((512*(1+a->tFlip)-a->tFlip*a->tibia)>430))
    return true;
  else{
    Serial.println("ERROR FUCK SHIT GTFO");
    return false;
  }
}
void writeServos(Leg * a) {
  SetPosition(a->coxaID,(int)a->coxa);
  SetPosition(a->femurID,(int)a->femur);
  SetPosition(a->tibiaID,(int)a->tibia);
}
void gait(float theta, int r,unsigned int tStep,unsigned int tReset,int mz){
  //generate next point
  // angle theta, step size r, tStep step time, tReset time between steps, mz magnitude z
  //pass to FR.setGoal(x,y,z), FL.setGoal() 
  tA=millis()%(tStep+tReset);
  tB=(millis()+(tStep+tReset)/2)%(tStep+tReset);
 if (tA <= tStep) {
    FR.setGoal(2*r*cos(theta)/2*((sin(0.5*(2*PI)/tStep*tA-PI/2))),
               2*r*sin(theta)/2*((sin(0.5*(2*PI)/tStep*tA-PI/2))),
               mz/2*((sin((2*PI)/tStep*tA-PI/2))+1));
    RL.setGoal(-2*r*cos(theta)/2*((sin(0.5*(2*PI)/tStep*tA-PI/2))),
               -2*r*sin(theta)/2*((sin(0.5*(2*PI)/tStep*tA-PI/2))),
               mz/2*((sin((2*PI)/tStep*tA-PI/2))+1));
  }
  else if (tA > tStep) {
    FR.setGoal(r*cos(theta)*(1-2.0/tReset*(tA-tStep)),//
               r*sin(theta)*(1-2.0/tReset*(tA-tStep)),
               0);
    RL.setGoal(-r*cos(theta)*(1-2.0/tReset*(tA-tStep)),
               -r*sin(theta)*(1-2.0/tReset*(tA-tStep)),
               0);
  }
  if (tB <= tStep) {
    FL.setGoal(2*r*cos(theta)/2*((sin(0.5*(2*PI)/tStep*tB-PI/2))),
               -2*r*sin(theta)/2*((sin(0.5*(2*PI)/tStep*tB-PI/2))),
               mz/2*((sin((2*PI)/tStep*tB-PI/2))+1));
    RR.setGoal(-2*r*cos(theta)/2*((sin(0.5*(2*PI)/tStep*tB-PI/2))),
               2*r*sin(theta)/2*((sin(0.5*(2*PI)/tStep*tB-PI/2))),
               mz/2*((sin((2*PI)/tStep*tB-PI/2))+1));
  }
  else if (tB > tStep) {
    FL.setGoal(r*cos(theta)*(1-2.0/tReset*(tB-tStep)),
               -r*sin(theta)*(1-2.0/tReset*(tB-tStep)),
               0);
    RR.setGoal(-r*cos(theta)*(1-2.0/tReset*(tB-tStep)),
               r*sin(theta)*(1-2.0/tReset*(tB-tStep)),
               0);
  }
}
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
void gait(float theta, int r,unsigned int tStep,unsigned int tReset,int mz);

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
  Leg FR (COXA_FR, FEMUR_FR, TIBIA_FR, -1, 1, 1);
  Leg FL (COXA_FL, FEMUR_FL, TIBIA_FL, 1, -1, -1);
  Leg RL (COXA_RL, FEMUR_RL, TIBIA_RL, -1, 1, 1);
  Leg RR (COXA_RR, FEMUR_RR, TIBIA_RR, 1, -1, -1);

//unsigned long int loopTime, lastTime;
void setup() {
  Serial.begin(38400);
  FR.setGoal(0,0,0);
  //loopTime = lastTime = 0;
}

int q = 0;

void loop() {
  //gait(theta,r,tStep,tReset,mz)
  //lastTime = millis();
  gait(0,40,250,270,20);
  //loopTime = millis() - lastTime;
  //Serial.println(loopTime);
  q++;
  if(q == 1) {
    for(int j = 0; j < 12; j++) {
      Serial.print(j+1);
      Serial.print(": ");
      Serial.print(ax12GetRegister(j+1, 43, 1));
      Serial.write(10);
    }
  }
  else if (q > 20)
    q = 0;


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
  if ( ((512*(1+a->cFlip)-a->cFlip*a->coxa) <720) && 
       ((512*(1+a->cFlip)-a->cFlip*a->coxa) >300) && 
       ((512*(1+a->fFlip)-a->fFlip*a->femur)<730) && 
       ((512*(1+a->fFlip)-a->fFlip*a->femur)>512)  && 
       ((512*(1+a->tFlip)-a->tFlip*a->tibia)<799) && 
       ((512*(1+a->tFlip)-a->tFlip*a->tibia)>430))
    return true;
  else{
    Serial.println("ERROR FUCK SHIT GTFO");
    return false;
  }
}
void writeServos(Leg * a) {
  SetPosition(a->coxaID,(int)a->coxa);
  SetPosition(a->femurID,(int)a->femur);
  SetPosition(a->tibiaID,(int)a->tibia);
}
void gait(float theta, int r,unsigned int tStep,unsigned int tReset,int mz){
  //generate next point
  // angle theta, step size r, tStep step time, tReset time between steps, mz magnitude z
  //pass to FR.setGoal(x,y,z), FL.setGoal() 
  tA=millis()%(tStep+tReset);
  tB=(millis()+(tStep+tReset)/2)%(tStep+tReset);
 if (tA <= tStep) {
    FR.setGoal(2*r*cos(theta)/2*((sin(0.5*(2*PI)/tStep*tA-PI/2))),
               2*r*sin(theta)/2*((sin(0.5*(2*PI)/tStep*tA-PI/2))),
               mz/2*((sin((2*PI)/tStep*tA-PI/2))+1));
    RL.setGoal(-2*r*cos(theta)/2*((sin(0.5*(2*PI)/tStep*tA-PI/2))),
               -2*r*sin(theta)/2*((sin(0.5*(2*PI)/tStep*tA-PI/2))),
               mz/2*((sin((2*PI)/tStep*tA-PI/2))+1));
  }
  else if (tA > tStep) {
    FR.setGoal(r*cos(theta)*(1-2.0/tReset*(tA-tStep)),//
               r*sin(theta)*(1-2.0/tReset*(tA-tStep)),
               0);
    RL.setGoal(-r*cos(theta)*(1-2.0/tReset*(tA-tStep)),
               -r*sin(theta)*(1-2.0/tReset*(tA-tStep)),
               0);
  }
  if (tB <= tStep) {
    FL.setGoal(2*r*cos(theta)/2*((sin(0.5*(2*PI)/tStep*tB-PI/2))),
               -2*r*sin(theta)/2*((sin(0.5*(2*PI)/tStep*tB-PI/2))),
               mz/2*((sin((2*PI)/tStep*tB-PI/2))+1));
    RR.setGoal(-2*r*cos(theta)/2*((sin(0.5*(2*PI)/tStep*tB-PI/2))),
               2*r*sin(theta)/2*((sin(0.5*(2*PI)/tStep*tB-PI/2))),
               mz/2*((sin((2*PI)/tStep*tB-PI/2))+1));
  }
  else if (tB > tStep) {
    FL.setGoal(r*cos(theta)*(1-2.0/tReset*(tB-tStep)),
               -r*sin(theta)*(1-2.0/tReset*(tB-tStep)),
               0);
    RR.setGoal(-r*cos(theta)*(1-2.0/tReset*(tB-tStep)),
               r*sin(theta)*(1-2.0/tReset*(tB-tStep)),
               0);
  }
}
