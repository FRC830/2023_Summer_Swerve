#include "WPISwerveDriveMotor.h"

void WPISwerveDriveMotor::Configure(SwerveDriveMotorConfig &config)
{
    SetVelocity(config.desired_vel);
    SetInverted(config.inverted);
    SetIdleMode(config.idleMode);

};

void WPISwerveDriveMotor::SetVelocity(double desired_vel)
{

};

double WPISwerveDriveMotor::GetVelocity()
{
    
};

void WPISwerveDriveMotor::SetInverted(bool inverted)
{
    
};

bool WPISwerveDriveMotor::GetInverted()
{
    
};

void WPISwerveDriveMotor::SetIdleMode(bool idleMode)
{
    m_idleMode = idleMode;
};

bool WPISwerveDriveMotor::GetIdleMode()
{
    return m_idleMode;
};
