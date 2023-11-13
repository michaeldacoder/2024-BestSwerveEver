#include "../../../include/swerve/Swerve.hpp"
#include <iostream>
#include <math.h>

void Swerve::clear_swerve_memory()
{
    for(int i = 0; i < 4; i++)
    {
        this->math_dest.wheel_angle[i] = 0;
        this->math_dest.wheel_speeds[i] = 0;
        this->raw_usable[i] = 0;
    }
}

Swerve::Swerve(float length, float width)
{
    this->chassis_info.length = length;
    this->chassis_info.width = width;

    clear_swerve_memory();

    for(int i = 0; i < 4; i++)
    {
        /* Set current limit */
        this->DRIVE_MOTORS[i]->SetSmartCurrentLimit(MAX_AMPERAGE);
        this->ANGLE_MOTORS[i]->SetSmartCurrentLimit(MAX_AMPERAGE);

        /* Turn on brake coast mode, snappier */
        this->DRIVE_MOTORS[i]->SetIdleMode(CANSparkMax::IdleMode::kBrake);
        this->ANGLE_MOTORS[i]->SetIdleMode(CANSparkMax::IdleMode::kBrake);

        /* Burn flash everytime (FUCK REV!!!) */
        this->DRIVE_MOTORS[i]->BurnFlash();
        this->ANGLE_MOTORS[i]->BurnFlash();

        /* PIDs */
        this->PID_CONTROLLERS[i] = new SparkMaxPIDController(this->ANGLE_MOTORS[i]->GetPIDController());
        this->PID_CONTROLLERS[i]->SetP(SWERVE_P);
        this->PID_CONTROLLERS[i]->SetI(SWERVE_I);
        this->PID_CONTROLLERS[i]->SetD(SWERVE_D);

        /* Get real relative encoders */
        this->ANGLE_ENCODERS[i] = new SparkMaxRelativeEncoder(this->ANGLE_MOTORS[i]->GetEncoder());
        std::cout << this->ANGLE_ENCODERS[i]->GetCountsPerRevolution() << "\n";
    }
};

void Swerve::calculate_wheel_information(wheel_info *dest, struct size_constants cons, float fwd, float str, float rotate, uint8_t field_centric, float gyro)
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

void Swerve::print_swerve_math(wheel_info math)
{
    std::cout << "\n";
    for(int i = 0; i < 4; i++)
    {
        std::cout << math.wheel_speeds[i] << " SPEED " << i << "\n";
        std::cout << math.wheel_angle[i] << " ANGLE " << i << "\n";
    }
}

/* When field centric mode is disabled 'gyro' is ignored */

void Swerve::drive(float y, float x, float x2, float gyro)
{
    bool y_deadzone = false;
    bool y_move_abs = false;

    /* Ignore our deadzone and fix the moving forward issue */
    if(y < DEADZONE_THRES && y > -DEADZONE_THRES)
    {
        y = 0;
        y_deadzone = true;
    }   
    if(x < DEADZONE_THRES && x > -DEADZONE_THRES)
    {
        x = 0;
        if(!y_deadzone)
        {
            /* Sets so that the true forward value is used incase of no strafing. Horrible hack but it works. */
            y_move_abs = true;
        }
    } 
    if(x2 < DEADZONE_THRES && x2 > -DEADZONE_THRES)
    {
        x2 = 0;
    }

    /* Generate our math and store in dest struct to be converted / used */
    calculate_wheel_information(&this->math_dest,this->chassis_info,y,x,x2,this->field_centered,gyro);

    int i;

    /* Allows for forward movement even when X is in deadzone */
    if(y_move_abs)
    {
        for(i = 0; i < 4; i++)
        {
            this->math_dest.wheel_speeds[i] = abs(y);
        }
    }

    /* Allows for sideways movement even when Y is in deadzone */
    if(y_deadzone)
    {
        for(int i = 0; i < 4; i++)
        {
            this->math_dest.wheel_speeds[i] = abs(x);
        }
    }

    /* Find the percent to max angle (180 or -180) and then multiple by the counts required to get to that required angle.      */
    /* Equivalent to x / SWERVE_WHEEL_COUNTS_PER_REVOLUTION = y / 360 where y is angle and x is raw sensor units for the encoder*/
    for(i = 0; i < 4; i++)
    { this->raw_usable[i] = ((this->math_dest.wheel_angle[i] / 360) * SWERVE_WHEEL_COUNTS_PER_REVOLUTION); }

    /* Only run our motors once everything is calculated */
    for(i = 0; i < 4; i++)
    {
        this->DRIVE_MOTORS[i]->Set(this->math_dest.wheel_speeds[i] * SWERVE_SPEED_MULTIPLIER);

        /* This stop it from going 360 around (all my michaels fault if it breaks)*/
        if((this->raw_usable[i] - this->ANGLE_ENCODERS[i]->GetPosition()) > 21)
        {
            this->raw_usable[i] = -(42-(this->raw_usable[i] - this->ANGLE_ENCODERS[i]->GetPosition()));
        }

        this->PID_CONTROLLERS[i]->SetReference(this->raw_usable[i],CANSparkMax::ControlType::kPosition);
       
        /* Print our raw encoder values */
        std::cout << i << "Actual: " << this->ANGLE_ENCODERS[i]->GetPosition() << " Desired: " << this->raw_usable[i] <<"\n";

        /* Clear "sticky" values that are stuck in memory, if the robot is receiving input this doesn't matter anyways.*/
        /* Only affects the robot when stopped!! */
        this->math_dest.wheel_speeds[i] = 0;
    }
    y_deadzone = false;
    y_move_abs = false;
}

bool Swerve::toggle_field_centricity()
{
    this->field_centered = !this->field_centered;
    return this->field_centered;
}