#pragma once
#include "SwerveDrive.h"

class WPISwerveDrive : public SwerveDrive
{
    public:
        virtual void Configure(SwerveConfig &config) = 0;
        virtual bool GetEbrake() override;
        virtual void SetEbrake(bool ebrake) override;
        virtual void Drive(frc::Translation2d position, double rotation) override;
        virtual void Drive(double vx, double vy, double omega) override;
        virtual void Drive(frc::ChassisSpeeds speed) override;
        virtual void Drive(std::vector<frc::SwerveModuleState> &state) override;
        virtual bool GetIdleMode() override;
        virtual void SetIdleMode(bool idle_mode) override;
        virtual void SetRobotOriented() override;
        virtual void SetFieldOriented() override;

    private:
        bool ebrake;
        bool orientation;

};