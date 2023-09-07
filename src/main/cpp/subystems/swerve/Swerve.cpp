#include "../../../include/swerve/Swerve.hpp"
#include <iostream>
#include <math.h>

#define F_PI 3.14159f

Swerve::Swerve(float length, float width)
{
    this->chassis_info.length = length;
    this->chassis_info.width = width;
    for(int i = 0; i < 4; i++)
    {
        /* Zero out our angle matrix initially */
        this->angle_matrix[i][0] = 0;
        this->angle_matrix[i][1] = 0;
        this->raw_usable_matrix[i] = 0;

        /* Config our angle motors using PID system */
        this->ANGLE_MOTORS[i].Config_kP(0, SWERVE_P);
        this->ANGLE_MOTORS[i].Config_kI(1, SWERVE_I);
        this->ANGLE_MOTORS[i].Config_kD(2, SWERVE_D);
    }
};

void Swerve::print_swerve_math(wheel_info math)
{
    std::cout << "\n";
    for(int i = 0; i < 4; i++)
    {
        std::cout << math.wheel_speeds[i] << " SPEED " << i << "\n";
        std::cout << math.wheel_angle[i] << " ANGLE " << i << "\n";
    }
}

/* When field centric mode is disabled 'gyro' is ignored             */

void Swerve::drive(float y, float x, float x2, float gyro)
{
    bool y_deadzone;
    bool y_move_abs;

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

    print_swerve_math(this->math_dest);

    /* Make our angles workable, taken from last year. Seems to work! */
    for(i = 0; i < 4; i++)
    {
        if(abs(this->angle_matrix[i][0] - this->math_dest.wheel_angle[i]) > F_PI && this->angle_matrix[i][0] < this->math_dest.wheel_angle[i])
        {
            this->angle_matrix[i][1] -= 2 * F_PI;
        }
        if(abs(this->angle_matrix[i][0] - this->math_dest.wheel_angle[i]) > F_PI && this->angle_matrix[i][0] > this->math_dest.wheel_angle[i])
        {
            this->angle_matrix[i][1] += 2 * F_PI;
        }

        /* Save new angle as previous */
        this->angle_matrix[i][0] = this->math_dest.wheel_angle[i];

        this->raw_usable_matrix[i] = -((this->math_dest.wheel_angle[i] + this->angle_matrix[i][1] / (F_PI * 2)) * SWERVE_WHEEL_COUNTS_PER_REVOLUTION);
    }

    /* Only run our motors once everything is calculated */

    for(i = 0; i < 4; i++)
    {
        this->DRIVE_MOTORS[i].Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, this->math_dest.wheel_speeds[i]);
        this->math_dest.wheel_speeds[i] = 0;
    }
    
    for(i = 0; i < 4; i++)
    {
        this->ANGLE_MOTORS[i].Set(ctre::phoenix::motorcontrol::ControlMode::Position, this->raw_usable_matrix[i]);
        this->raw_usable_matrix[i] = 0;
    }

    y_deadzone = false;
    y_move_abs = false;
}

bool Swerve::toggle_field_centricity()
{
    this->field_centered = !this->field_centered;
    return this->field_centered;
}