#include "../../../include/swerve/Swerve.hpp"

Swerve::Swerve(float length, float width)
{
    this->chassis_info.length = length;
    this->chassis_info.width = width;
};

/* NOTE: do NOT reverse the fwd (x) polarity! It should be negative! */
/* When field centric mode is disabled 'gyro' is ignored             */

void Swerve::drive(float x, float y, float z, float gyro)
{
    /* Generate our math and store in dest struct to be converted / used */
    calculate_wheel_information(&this->math_dest,this->chassis_info,x,y,z,this->field_centered,gyro);
    
    /* Move our swerve based on calculations */
    int i;
    for(i = 0; i < 4; i++)
    {
        this->DRIVE_MOTORS[i].Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, this->math_dest.wheel_speeds[i]);
    }
    
    for(i = 0; i < 4; i++)
    {
        this->ANGLE_MOTORS->Set(ctre::phoenix::motorcontrol::ControlMode::Position, this->math_dest.wheel_angle[i]);
    }
}

bool Swerve::toggle_field_centricity()
{
    this->field_centered = !this->field_centered;
    return this->field_centered;
}