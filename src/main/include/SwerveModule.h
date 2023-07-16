#pragma once
#include <frc/kinematics/SwerveModuleState.h>


struct SwerveModuleConfig;


class SwerveModule
{
public:
    virtual void SetState(frc::SwerveModuleState state) = 0; 
    virtual frc::SwerveModuleState GetState() = 0;
    virtual void SetIdleMode() = 0;
    virtual bool GetIdleMode() = 0;
    virtual void Configure(SwerveModuleConfig &config) = 0;
private:


};
