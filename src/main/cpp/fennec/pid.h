#pragma once

struct PID
{
	float kP; //Proportional 
	float kI; //Integral
	float kD; //Derivative

//Hehe PID more like PIG

	float errorAccum = 0;
	float lastError = 0;

	bool first = false;
};

void initPid(PID* pid);
float evalPid(PID* pid, float error, float dt);