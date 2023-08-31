#include "../../../include/swerve/swerve_math.h"
#include <math.h>

/* Written by Zane Maples, 7/31/23 */
/* Meant to be used with a 4 wheel */
/* swerve drive. Float used over   */
/* double for speed.               */

/* Compile with -lm for math lib   */

void calculate_wheel_information(wheel_info *dest, struct size_constants cons, float fwd, float str, float rotate, uint8_t field_centric, float gyro)
{
	float forward = fwd;
	float strafe = str;

	if(field_centric)
	{
		/* If field centric take in account the gyro */
		forward = fwd * cosf(gyro) + str * sinf(gyro);
		strafe = -fwd * sinf(gyro) + str * cosf(gyro);
	}

	float R = sqrtf((cons.length * cons.length)+(cons.width * cons.width));

	float A, B, C, D;
	A = strafe - rotate * (cons.length/R);
	B = strafe + rotate * (cons.length/R);
	C = forward - rotate * (cons.width/R);
	D = forward + rotate * (cons.width/R);

	/* Fill out our dest struct */
	
	float a_sqrd, b_sqrd, c_sqrd, d_sqrd;
	a_sqrd = A*A;
	b_sqrd = B*B;
	c_sqrd = C*C;
	d_sqrd = D*D;

	float wheel_speeds[4];

	wheel_speeds[0] = sqrtf((b_sqrd) + (c_sqrd));
	wheel_speeds[1] = sqrtf((b_sqrd) + (d_sqrd));
	wheel_speeds[2] = sqrtf((a_sqrd) + (d_sqrd));
	wheel_speeds[3] = sqrtf((a_sqrd) + (c_sqrd));

	float max = 0;

	/* Do normalisation on the wheel speeds, so that they are 0.0 <-> +1.0 */
	int i;
	for(i = 0; i < 4; i++)
	{
		if(wheel_speeds[i] > max) { max = wheel_speeds[i]; }
	}

	if(max > 1)
	{
		for(i = 0; i < 4; i++) { dest->wheel_speeds[i] = wheel_speeds[i] / max; }
	}

	dest->wheel_angle[0] = atan2f(B,C) * MAGIC_NUMBER;
	dest->wheel_angle[1] = atan2f(B,D) * MAGIC_NUMBER;
	dest->wheel_angle[2] = atan2f(A,D) * MAGIC_NUMBER;
	dest->wheel_angle[3] = atan2f(A,C) * MAGIC_NUMBER;
	
	return;
}
