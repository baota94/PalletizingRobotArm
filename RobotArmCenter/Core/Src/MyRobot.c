#include "MyRobot.h"
#include "math.h"

void Robot_Init(Robot *robot, float maxChangeThetaStep, float maxChange, uint16_t minPulse, uint16_t maxPulse) {
    robot->maxChangeThetaStep = maxChangeThetaStep;
    robot->maxChange = maxChange;
    robot->minPulse = minPulse;
    robot->maxPulse = maxPulse;
    robot->stepCount = 0;
    robot->ProcessFlag = 0;
    robot->InitError = 0;
    robot->ForwardKinematicError = 0;
    robot->InverseKinematicError = 0;
    robot->ProcessError = 0;
    for(int i = 0; i < 8; i++) {
        robot->PathList[i] = i;
    }
}
void Robot_ForwardKinematics(Robot *robot) {
    // Implement the forward kinematics calculations here
    // This is a placeholder for the actual implementation
    float t234 = robot->fTheta[1] + robot->fTheta[2] + robot->fTheta[3];
    float t23 = robot->fTheta[1] + robot->fTheta[2];
    float cos_t234 = cosf(t234);
    float sin_t234 = sinf(t234);
    float cos_t2 = cosf(robot->fTheta[1]);
    float sin_t2 = sinf(robot->fTheta[1]);
    float cos_t1 = cosf(robot->fTheta[0]);
    float sin_t1 = sinf(robot->fTheta[0]);

    float arm_proj = 6 * cos_t234 - sin_t234 + 4 * cosf(t23) + 4 * cos_t2;

    float x = 25 * cos_t1 * arm_proj;
    float y = 25 * sin_t1 * arm_proj;
    float z = 100 * sin_t2 + 100 * sinf(t23) + 25 * sqrtf(37) * cosf(t234 - atanf(6));

    float roll = (M_PI * ((cos_t234 >= 0) ? 1 : -1)) / 2;
    float pitch = atan2f(-sin_t234, sqrtf(cos_t234 * cos_t234));

    float t1234 = robot->fTheta[0] + t234;
    float t_1_234 = -robot->fTheta[0] + t234;
    float yaw = atan2f((sinf(t1234) - sinf(t_1_234)) / 2, (cosf(t1234) + cosf(t_1_234)) / 2);

    robot->fPos[0] = x;
    robot->fPos[1] = y;
    robot->fPos[2] = z;
    robot->fPos[3] = roll;
    robot->fPos[4] = pitch;
    robot->fPos[5] = yaw;
    robot->ForwardKinematicError = 0; // Set to 0 if no error occurs
}
void Robot_InverseKinematics(Robot *robot) {
    // Implement the inverse kinematics calculations here
    // This is a placeholder for the actual implementation
    float r = sqrtf(robot->fPosTarget[0] * robot->fPosTarget[0] + robot->fPosTarget[1] * robot->fPosTarget[1]);
    float pwr = r - 25.0f;
    float pwz = robot->fPosTarget[2] + 150.0f;
    float a = sqrtf(pwr * pwr + pwz * pwz);

    // Round to 1 decimal place
    a = roundf(a * 10.0f) / 10.0f;

    // Check reachability
    if (a > 200.0f) {
        robot->InverseKinematicError = 1; // Set error flag
        return;
    } else if (a == 200.0f) {
        robot->fThetaTarget[3] = 0.0f;
    } else if (a == 0.0f) {
        robot->fThetaTarget[3] = M_PI;
    }

    // Normal calculation if not at boundaries
    if (!(a == 200.0f || a == 0.0f)) {
        robot->fThetaTarget[3] = acosf((a * a - 10000.0f - 10000.0f) / (-2.0f * 100.0f * 100.0f));
    }

    float alpha = (M_PI - robot->fThetaTarget[3]) / 2.0f;
    robot->fThetaTarget[3] = M_PI - robot->fThetaTarget[3];

    robot->fThetaTarget[2] = M_PI - (atan2f(pwz, pwr) + acosf((10000.0f + a * a - 10000.0f) / (2.0f * 100.0f * a)));
    robot->fThetaTarget[4] = M_PI - (atan2f(pwr, pwz) + alpha);

    robot->fThetaTarget[1] = atan2f(robot->fPosTarget[1], robot->fPosTarget[0]);
    if (robot->fThetaTarget[1] < 0.0f) {
        robot->fThetaTarget[1] += M_PI;
    }
    robot->InverseKinematicError = 0; // Set to 0 if no error occurs
}
void Robot_Process(Robot *robot) {
    // Implement the process logic here
    // This is a placeholder for the actual implementation
    switch (robot->Mode) {
        case _SET_ANGLE_:
            // Code to set the angle
            //printf("Setting angle...\n");
            robot->ProcessPoint = _SET_ANGLE_; // Set to _SET_ANGLE ensure the case correct
            // Add logic to set angles here
            Robot_Run(robot); // Call the run function to process the angles
            break;

        case _SET_POSITION_:
            // Code to set the position
            robot->ProcessPoint = _SET_POSITION_; // Set to _SET_POSITION_ ensure the case correct
            // Add logic to set position here
            Robot_InverseKinematics(robot); // Call inverse kinematics to calculate angles
            Robot_Run(robot); // Call the run function to process the angles
            break;

        case _RUNROBOT_:
            // Code to run the robot
            robot->ProcessPoint = _RUNROBOT_; // Set to _RUNROBOT_ ensure the case correct
            // Add logic to execute robot movement here
            Robot_RunPath(robot); // Call the run function to process the path
            break;

        case _STOPROBOT_:
            // Code to stop the robot
            robot->ProcessPoint = _STOPROBOT_; // Set to _STOPROBOT_ ensure the case correct
            Robot_Stop(robot);
            break;

        case _RESETROBOT_:
            // Code to reset the robot
            robot->ProcessPoint = _RESETROBOT_; // Set to _RESETROBOT_ ensure the case correct
            Robot_Reset(robot);
            break;

        default:
            printf("Invalid process flag\n");
            robot->ProcessError = 1; // Set error flag
            break;
    }
    robot->ProcessFlag = 1; // Set to 1 if processing is done
}
void Robot_Reset(Robot *robot) {
    robot->stepCount = 0;
    robot->ProcessFlag = 0;
    robot->InitError = 0;
    robot->ForwardKinematicError = 0;
    robot->InverseKinematicError = 0;
    robot->ProcessError = 0;
    for (int i = 0; i < 6; i++) {
        robot->fTheta[i] = PI_2;
        robot->fThetaTarget[i] = PI_2;
    }
    Robot_SetPulse(robot); // Set pulse values to default
}
void Robot_Stop(Robot *robot) {
    robot->ProcessFlag = 0;
    for (int i = 0; i < 6; i++) {
        robot->fThetaTarget[i] = robot->fTheta[i];
    }
}
void Robot_Start(Robot *robot) {
    robot->ProcessFlag = 1;
}
void Robot_DeInit(Robot *robot) {
    robot->stepCount = 0;
    robot->ProcessFlag = 0;
    robot->InitError = 0;
    robot->ForwardKinematicError = 0;
    robot->InverseKinematicError = 0;
    robot->ProcessError = 0;
}
void Robot_Run(Robot *robot) {
    // Implement the run logic here
    // This is a placeholder for the actual implementation
    while (robot->ProcessFlag) {
    }
    float maxDifference = 0.0f;
    float difference[6];
    for (int i = 0; i < 6; i++) {
        difference[i] = (robot->fThetaTarget[i] - robot->fTheta[i]);
        if (difference[i] > maxDifference) {
            maxDifference = fabsf(difference[i]);
        }
    }
    if (maxDifference > robot->maxChangeThetaStep) {
        for (int i = 0; i < 6; i++) {
            robot->Step[i] = (robot->fThetaTarget[i] - robot->fTheta[i]) / maxDifference;
            robot->fTheta[i] += (difference[i]) * robot->maxChangeThetaStep / maxDifference;
        }
    } else {
        for (int i = 0; i < 6; i++) {
            robot->fTheta[i] = robot->fThetaTarget[i];
        }
    }
    Robot_ForwardKinematics(robot); // Update forward kinematics
    Robot_SetPulse(robot); // Set pulse values based on updated angles
    robot->ProcessFlag = 1; // Reset process flag after running
}
void Robot_SetPulse(Robot *robot) {
    // Implement the pulse setting logic here
    // This is a placeholder for the actual implementation
    for (int i = 0; i < 6; i++) {
        float theta = robot->fThetaTarget[i];
        if (theta < -PI || theta > PI) {
            robot->ProcessError = 1; // Set error flag
            return;
        }
        robot->pulse[i] = (uint16_t)((theta + PI) * (robot->maxPulse - robot->minPulse) / (2 * PI) + robot->minPulse);
    }
}
void Robot_RunPath(Robot *robot) {
    // Implement the path running logic here
    // This is a placeholder for the actual implementation
    uint8_t posFinished = 0;
    for(int i = 0; i < 6; i++) {
        robot->fThetaTarget[i] = robot->PosList[robot->PathList[robot->PathIndex]][i];
    }
    posFinished = 1;
    for (int j = 0; j < 6; j++) {
        if (fabsf(robot->fTheta[j] - robot->fThetaTarget[j]) > 1e-3) {
            posFinished = 0;
            break;
        }
    } 
    if(posFinished) {
        robot->PathIndex++;
        if (robot->PathIndex >= 8) {
            robot->PathIndex = 0;
            robot->PathList[6] += 2;
            robot->PathList[7] += 2;
        }
    }
    else {
        Robot_Run(robot); // Call the run function to process the path
    }
}
