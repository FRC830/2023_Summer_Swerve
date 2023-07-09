#pragma once
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <vector>

struct SwerveConfig;

class SwerveDrive 
{
    public:
        virtual void Configure(SwerveConfig &config) = 0;
        virtual bool GetEbrake() = 0;
        virtual void SetEbrake(bool ebrake) = 0;
        virtual void Drive(double x_position, double y_position, double rotation) = 0;
        virtual void Drive(double vx, double vy, double omega) = 0;
        virtual void Drive(frc::ChassisSpeeds speed) = 0;
        virtual void Drive(std::vector<frc::SwerveModuleState> &state) = 0;
        virtual bool GetIdleMode() = 0;
        virtual void SetIdleMode(bool idle_mode) = 0;
        virtual void SetRobotOriented() = 0;
        virtual void SetFieldOriented() = 0;
        virtual bool GetOrientedMode() = 0; 

    private:

};
