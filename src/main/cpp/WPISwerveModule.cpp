#include "WPISwerveModule.h"

void WPISwerveModule::Configure(SwerveModuleConfig &config)
{
    SetIdleMode(config.idleMode);
};

void WPISwerveModule::SetState(frc::SwerveModuleState state)
{

    
};

frc::SwerveModuleState WPISwerveModule::GetState()
{
    
};

void WPISwerveModule::SetIdleMode(bool idleMode)
{
    m_idleMode = idleMode;
};

bool WPISwerveModule::GetIdleMode()
{
    return m_idleMode;
};
