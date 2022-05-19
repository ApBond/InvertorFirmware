#include "kinematic.h"

double sign(double a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 0;
}

int16_t intSign(double a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 0;
}

void kinematica(rcCommand_t cmd, float* Speed, float* targetAngle){
	float x,y;
	float R[4];  
	float Rm,gm;
	float gam,V0,R0;
	gam=atan2(cmd.Vy,cmd.Vx);
    V0=sqrt(powf(cmd.Vx,2)+powf(cmd.Vy,2));
	R0=V0/cmd.wz;
	x=R0*cos(pi/2+gam);
	y=R0*sin(pi/2+gam);

	if (cmd.wz>0){
		targetAngle[FL]=atan2(-x+L/2, y-C/2);
		targetAngle[FR]=atan2(-x+L/2, y+C/2);
		targetAngle[RL]=atan2(-x-L/2, y-C/2);
		targetAngle[RR]=atan2(-x-L/2, y+C/2);
	}
	else if (cmd.wz<0){
		targetAngle[FL]=-atan2(-x+L/2, -y+C/2);
		targetAngle[FR]=-atan2(-x+L/2, -y-C/2);
		targetAngle[RL]=-atan2(-x-L/2, -y+C/2);
		targetAngle[RR]=-atan2(-x-L/2, -y-C/2);
	}
	else{
		targetAngle[FL]=gam;
		targetAngle[FR]=gam;
		targetAngle[RL]=gam;
		targetAngle[RR]=gam;
	}

	R[FL]=sqrt(powf(L/2-x,2)+powf(C/2-y,2));
	R[FR]=sqrt(powf(L/2-x,2)+powf(-C/2-y,2));
	R[RL]=sqrt(powf(-L/2-x,2)+powf(C/2-y,2));
	R[RR]=sqrt(powf(-L/2-x,2)+powf(-C/2-y,2));

	if (fabs(targetAngle[FL])<0.01) targetAngle[FL]=0;
	if (fabs(targetAngle[FR])<0.01) targetAngle[FR]=0;
	if (fabs(targetAngle[RL])<0.01) targetAngle[RL]=0;
	if (fabs(targetAngle[RR])<0.01) targetAngle[RR]=0;

	targetAngle[FR] = -targetAngle[FR];
	targetAngle[RL] = -targetAngle[RL];

    if (cmd.wz==0){
		(*Speed)=V0*60/r_wheel/2/pi;
	}
    else if (V0==0){
        (*Speed)=(cmd.wz*60/r_wheel/2/pi)*R[WHEEL];
	}
    else{
        (*Speed)=(((V0*60/r_wheel/2/pi)))*R[WHEEL]/R0;
	}

}
