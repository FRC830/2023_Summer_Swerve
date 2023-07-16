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
    return m_desired_vel;
};

void WPISwerveDriveMotor::SetInverted(bool inverted)
{
    m_inverted = inverted;
};

bool WPISwerveDriveMotor::GetInverted()
{
    return m_inverted;
};

void WPISwerveDriveMotor::SetIdleMode(bool idleMode)
{
    m_idleMode = idleMode;
};

bool WPISwerveDriveMotor::GetIdleMode()
{
    return m_idleMode;
};