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
    calculate_wheel_information(&this->math_dest,this->chassis_info,x,y,z,this->field_centred,gyro);
    
    /* Wheel speeds are normalised to 0-1 percent anyways. No need for conversion promise! */
    int i;
    for(i = 0; i < 4; i++)
    {
        this->DRIVE_MOTORS[i].Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, this->math_dest.wheel_speeds[i]);
    }
    
    /* Normalise wheel angles by 180 (this probably won't work lol but maybe just maybe)*/
    for(i = 0; i < 4; i++)
    {
        this->ANGLE_MOTORS->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, this->math_dest.wheel_angle[i]/180);
    }
}