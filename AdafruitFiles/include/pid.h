#ifndef PID_H
#define PID_H

extern float kpRoll;
extern float kiRoll;
extern float kdRoll;
extern float kpPitch;
extern float kiPitch;
extern float kdPitch;
extern float kpYaw;
extern float kiYaw;
extern float kdYaw;
extern float integralLimit;

extern float rollPid;
extern float pitchPid;
extern float yawPid;
extern float integralRoll;
extern float integralPitch;
extern float integralYaw;

extern float rollDesired;
extern float pitchDesired;
extern float yawDesired;

void calculatePID();

#endif
