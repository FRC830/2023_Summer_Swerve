#pragma once
#include "Interfaces/SwerveModule.h"

struct SwerveModuleConfig
{
    bool idleMode;

};

class WPISwerveModule : SwerveModule
{
public:
    WPISwerveModule() = default;
    virtual ~WPISwerveModule() = default;
    virtual void SetState(frc::SwerveModuleState state) override; 
    virtual frc::SwerveModuleState GetState() override;
    virtual void SetIdleMode(bool idleMode) override;
    virtual bool GetIdleMode() override;
    virtual void Configure(SwerveModuleConfig &config) override;

private:
    bool m_idleMode;

};
