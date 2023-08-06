#pragma once

#include <rev/CANSparkMAX.h>
#include "SwerveDriveMotor.h"
#include "units/velocity.h"

struct SwerveDriveMotorConfig {
    int motorID;
    bool inverted;
    rev::CANSparkMax *motor;
    rev::SparkMaxRelativeEncoder *encoder;
    rev::SparkMaxPIDController *PID;
    rev::CANSparkMax::IdleMode idleMode; 
    double p;
    double i;
    double d;
    double ff;
    double ratio;
    units::velocity::meters_per_second_t MaxSpeed;
    

};

class NeoDriveMotor : SwerveDriveMotor {
    
    public:
        NeoDriveMotor(); 
        virtual ~NeoDriveMotor();
        virtual void Configure(SwerveDriveMotorConfig &config) override; 
        virtual void SetVelocity(units::velocity::feet_per_second_t v) override; 
        virtual double GetVelocity() override; 
        virtual void SetInverted(bool isInverted) override;
        virtual bool GetInverted() override;
        virtual void SetIdleMode(rev::CANSparkMax::IdleMode m) override;
        virtual rev::CANSparkMax::IdleMode GetIdleMode() override; // just incase if we use motors other than the rev ones that use otehr than the rev stuff. idk 




    private: 
        int m_motorID;
        bool m_inverted;
        rev::CANSparkMax *m_motor;
        rev::SparkMaxRelativeEncoder *m_encoder;
        rev::SparkMaxPIDController *m_PID;
        units::velocity::meters_per_second_t m_MaxSpeed; 
        




};