#ifndef SWERVE_MATH_H
#define SWERVE_MATH_H
#include <stdint.h>

/* 180 / Pi */
#define MAGIC_NUMBER 57.29577f

struct wheel_information
{
	/* Assuming one wheel in each corner */
	
	/* Order of wheels as in the real world: */
	/* 0 = front right, 1 = front left       */
	/* 2 = rear left,   3 = rear right       */

	float wheel_speeds[4]; 
	int wheel_angle[4];
};

typedef struct wheel_information wheel_info;

struct size_constants
{
	float length;
	float width;
};

void calculate_wheel_information(wheel_info *dest, struct size_constants cons, float fwd, float str, float rotate, uint8_t field_centric, float gyro);
#endif