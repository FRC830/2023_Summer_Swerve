#pragma once
#include "SwerveDrive.h"

struct SwerveConfig{
    bool idle_mode;
    bool ebrake;
    bool orientation;
    double maxDriveSpeed;
    double maxTurnSpeed;
    //location of motors relative to the centor of the robot
 



};

class WPISwerveDrive : public SwerveDrive
{
    public:
        virtual void Configure(SwerveConfig &config) = 0;
        virtual bool GetEbrake() override;
        virtual void SetEbrake(bool ebrake) override;
        virtual void Drive(double x_position, double y_position, double rotation);
        virtual void Drive(double vx, double vy, double omega) override;
        virtual void Drive(frc::ChassisSpeeds speed) override;
        virtual void Drive(std::vector<frc::SwerveModuleState> &state) override;
        virtual bool GetIdleMode() override;
        virtual void SetIdleMode(bool idle_mode) override;
        virtual void SetRobotOriented() override;
        virtual void SetFieldOriented() override;
        virtual bool GetOrientedMode() override; 



    private:

        frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
        frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
        frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};
        frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};

        std::vector<frc::SwerveModuleState> states;
        
        double m_maxDriveSpeed;
        double m_maxTurnSpeed;
        bool m_ebrake = false;
        bool m_driveMotorIdleMode = false;
        //false is brake and true is coast
        bool m_orientation = false;
        //false is robot orientated, true is FieldOrientated. 
        


};