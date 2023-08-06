#pragma once
#include "Interfaces/SwerveDriveMotor.h"

struct SwerveDriveMotorConfig{
    bool inverted;
    bool idleMode;

};

class WPISwerveDriveMotor : SwerveDriveMotor{
public:
    virtual void Configure(SwerveDriveMotorConfig &config) override;
    virtual void SetVelocity(double desired_vel) override;
    virtual double GetVelocity() override;
    virtual void SetInverted(bool inverted) override;
    virtual bool GetInverted() override;
    virtual void SetIdleMode(bool idleMode) override;
    virtual bool GetIdleMode() override;

private:
    bool m_inverted;
    bool m_idleMode;

};