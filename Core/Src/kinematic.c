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
	x=cmd.R*cos(pi/2+cmd.gam);
	y=cmd.R*sin(pi/2+cmd.gam);
	if (cmd.R>0){
		targetAngle[FL]=atan2(L/2-x, y-C/2);
		targetAngle[FR]=atan2(L/2-x, y+C/2);
		targetAngle[RL]=atan2(-L/2-x, y-C/2);
		targetAngle[RR]=atan2(-L/2-x, y+C/2);
	}
	else{
		targetAngle[RR]=atan2(L/2+x, -y-C/2);
		targetAngle[RL]=atan2(L/2+x, -y+C/2);
		targetAngle[FR]=atan2(-L/2+x, -y-C/2);
		targetAngle[FL]=atan2(-L/2+x, -y+C/2);
	}

	R[FL]=fabs(y-C/2)/fabs(cos(targetAngle[FL]));
	R[FR]=fabs(y+C/2)/fabs(cos(targetAngle[FR]));
	R[RL]=fabs(y-C/2)/fabs(cos(targetAngle[RL]));
	R[RR]=fabs(y+C/2)/fabs(cos(targetAngle[RR]));

	if(R[FL]>1000) R[FL]=100000;
	if(R[FR]>1000) R[FR]=100000;
	if(R[RL]>1000) R[RL]=100000;
	if(R[RR]>1000) R[RR]=100000;
	Rm=R[FL];
	if (Rm<R[FR]) Rm=R[FR];
	if (Rm<R[RL]) Rm=R[RL];
	if (Rm<R[RR]) Rm=R[RR];

	if (fabs(targetAngle[FL])<0.01) targetAngle[FL]=0;
	if (fabs(targetAngle[FR])<0.01) targetAngle[FR]=0;
	if (fabs(targetAngle[RL])<0.01) targetAngle[RL]=0;
	if (fabs(targetAngle[RR])<0.01) targetAngle[RR]=0;

	(*Speed)=(((cmd.V*60/r_wheel/2/pi)))*R[WHEEL]/Rm;
}
