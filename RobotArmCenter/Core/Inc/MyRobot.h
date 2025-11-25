#ifndef _MYROBOT_H_
#define _MYROBOT_H_
#include "main.h"
#define PI 3.14159265358979323846
#define PI2 6.28318530717958647692
#define PI_2 1.57079632679489661923
#define PI_4 0.78539816339744830962
#define PI_6 0.52359877559829887308
#define PI_3 1.04719755119659774615
#define PI_8 0.39269908169872415481

typedef struct {
    float fTheta[6];
    float fThetaTarget[6];
    float fPos[6];
    float fPosTarget[6];
    uint16_t pulse[6];
    uint16_t minPulse;
    uint16_t maxPulse;
    float maxChangeThetaStep;
    float maxChange;
    float Step[6];
    uint16_t stepCount;
    uint8_t initError;
    uint8_t CalculateError;
    uint16_t ProcessError;
    uint8_t ProcessPoint;
    uint8_t ForwardKinematicError;
    uint8_t InverseKinematicError;
    uint8_t ProcessFlag;
    uint8_t Mode;
    uint8_t InitError;
    uint16_t PathList[8];
    uint16_t PathIndex;
    uint8_t PosList[100][6];
    uint8_t PosIndex;
} Robot;
void Robot_Init(Robot *robot, float maxChangeThetaStep, float maxChange, uint16_t minPulse, uint16_t maxPulse);
void Robot_ForwardKinematics(Robot *robot);
void Robot_InverseKinematics(Robot *robot);
void Robot_Process(Robot *robot);
void Robot_Reset(Robot *robot);
void Robot_Stop(Robot *robot);
void Robot_DeInit(Robot *robot);
void Robot_Run(Robot *robot);
void Robot_SetPulse(Robot *robot);
void Robot_RunPath(Robot *robot);
#endif // _MYROBOT_H_
 
