#pragma once

#include <frc/geometry/Rotation2d.h>
#include "units/velocity.h"
#include <rev/CANSparkMax.h>

struct SwerveDriveMotorConfig;


class SwerveDriveMotor{
    public: 
        SwerveDriveMotor() = default;
        virtual ~SwerveDriveMotor() = default;
        virtual void Configure(SwerveDriveMotorConfig &config) = 0;
        virtual void SetVelocity(units::velocity::feet_per_second_t v) = 0; 
        virtual double GetVelocity() = 0;
        virtual void SetInverted(bool isInverted) = 0;
        virtual bool GetInverted() = 0;
        virtual void SetIdleMode(rev::CANSparkMax::IdleMode m) = 0;
        virtual rev::CANSparkMax::IdleMode GetIdleMode() = 0; // just incase if we use motors other than the rev ones that use otehr than the rev stuff. idk 


    private: 

 




};