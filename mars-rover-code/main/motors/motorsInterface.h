#ifndef MOTORS_H
#define MOTORS_H

#ifdef __cplusplus
extern "C" {
#endif

void moveForward(void);
void moveBackward(void);
void turnLeftMotors(void);
void turnRightMotors(void);
void stopMotors(void);
void frontServoLeft(void);
void frontServoRight(void);
void backServoLeft(void);
void backServoRight(void);

#ifdef __cplusplus
}
#endif

#endif