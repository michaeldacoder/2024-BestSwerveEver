#include "../../../include/swerve/Swerve.hpp"
#include <iostream>
#include <math.h>

Swerve::Swerve(float length, float width)
{
    this->chassis_info.length = length;
    this->chassis_info.width = width;
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

/* NOTE: do NOT reverse the fwd (x) polarity! It should be negative! */
/* When field centric mode is disabled 'gyro' is ignored             */

void Swerve::drive(float y, float x, float gyro)
{
    bool x_deadzone;
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
        x_deadzone = true;
    } 

    float z = 0;

    /* Generate our math and store in dest struct to be converted / used */
    calculate_wheel_information(&this->math_dest,this->chassis_info,y,x,z,this->field_centered,gyro);
    print_swerve_math(this->math_dest);

    /* If idle just return and DO NOT feed the motors 
    TODO: this needs to be tested. could potentially just constantly move motors if not properly cleaned */
    if(y_deadzone && x_deadzone)
    {
        return;
    }

    /* Move our swerve based on calculations, then flush data */
    int i;

    /* Fixes our issues of forward / backward with no speed */
    if(y_move_abs)
    {
        for(i = 0; i < 4; i++)
        {
            this->math_dest.wheel_speeds[i] = y;
            std::cout << "foward ";
        }
    }

    for(i = 0; i < 4; i++)
    {
        this->DRIVE_MOTORS[i].Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, this->math_dest.wheel_speeds[i]);
        this->math_dest.wheel_speeds[i] = 0;
    }
    
    for(i = 0; i < 4; i++)
    {
        this->ANGLE_MOTORS->Set(ctre::phoenix::motorcontrol::ControlMode::Position, this->math_dest.wheel_angle[i]);
    }

    x_deadzone = false;
    y_deadzone = false;
    y_move_abs = false;
}



bool Swerve::toggle_field_centricity()
{
    this->field_centered = !this->field_centered;
    return this->field_centered;
}