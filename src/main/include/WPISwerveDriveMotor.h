#pragma once
#include "SwerveDriveMotor.h"

struct SwerveDriveMotorConfig{
    double desired_vel;
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
    double m_desired_vel;
    bool m_inverted;
    bool m_idleMode;

};