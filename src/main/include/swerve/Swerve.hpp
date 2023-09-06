#ifndef SWERVE_H
#define SWERVE_H

/* CAN IDS */
#define FR_M 1
#define FL_M 2
#define RL_M 3
#define RR_M 4

#define FR_A 5
#define FL_A 6
#define RL_A 7
#define RR_A 8

/* Swerve Constants */
#define DEADZONE_THRES .005

#include <frc2/command/SubsystemBase.h>
#include "swerve_math.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

using namespace ctre::phoenix::motorcontrol::can;

class Swerve : frc2::SubsystemBase
{
    public:
        bool field_centered = false;
        Swerve(float length, float width);
        void drive(float x, float y, float gyro);
        void print_swerve_math(wheel_info math);
        bool toggle_field_centricity(); // returns changed state
    private:
        wheel_info math_dest;
        struct size_constants chassis_info;

        /* Motor bank. Follows the format in the math_dest 
            0 = front right, 1 = front left 
            2 = rear left,   3 = rear right */
        
        // feast upon this awesome code!! (yes it works)
        WPI_TalonFX DRIVE_MOTORS[4] = 
        {
            {FR_M},
            {FL_M},
            {RL_M},
            {RR_M}
        };
        WPI_TalonFX ANGLE_MOTORS[4] = 
        {
            {FR_A},
            {FL_A},
            {RL_A},
            {RR_A}
        };
};

#endif