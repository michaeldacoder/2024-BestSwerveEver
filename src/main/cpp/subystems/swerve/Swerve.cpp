#include "../../../include/swerve/Swerve.hpp"
#include <iostream>

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

void Swerve::drive(float y, float x, float z, float gyro)
{
    /* Ignore our deadzone */
    if(y < .5 && y > -.5)
    {
        y = 0;
    }
    if(x < .5 && x > -.5)
    {
        x = 0;
        std::cout << "X Deadzone Reached: " << x;
    }
    
    /* Generate our math and store in dest struct to be converted / used */
    calculate_wheel_information(&this->math_dest,this->chassis_info,y,x,z,this->field_centered,gyro);
    print_swerve_math(this->math_dest);

    /* Move our swerve based on calculations, then flush data */
    int i;
    for(i = 0; i < 4; i++)
    {
        this->DRIVE_MOTORS[i].Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, this->math_dest.wheel_speeds[i]);
        this->math_dest.wheel_speeds[i] = 0;
    }
    
    for(i = 0; i < 4; i++)
    {
        this->ANGLE_MOTORS->Set(ctre::phoenix::motorcontrol::ControlMode::Position, this->math_dest.wheel_angle[i]);
        this->math_dest.wheel_angle[i] = 0;
    }
}



bool Swerve::toggle_field_centricity()
{
    this->field_centered = !this->field_centered;
    return this->field_centered;
}