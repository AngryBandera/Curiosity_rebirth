#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void servos_init(void);

void backServoRight(void);
void backServoLeft(void);
void frontServoRight(void);
void frontServoLeft(void);

#ifdef __cplusplus
}
#endif