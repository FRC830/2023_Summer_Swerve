#pragma once

struct SwerveDriveMotorConfig;

class SwerveDriveMotor
{
public:
    virtual void Configure(SwerveDriveMotorConfig &config) = 0;
    virtual void SetVelocity(double desired_vel) = 0;
    virtual double GetVelocity() = 0;
    virtual void SetInverted(bool inverted) = 0;
    virtual bool GetInverted() = 0;
    virtual void SetIdleMode(bool idleMode) = 0;
    virtual bool GetIdleMode() = 0;

private:


};