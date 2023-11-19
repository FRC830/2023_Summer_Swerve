#include "WPISwerveDrive.h"
#include "frc/Timer.h"

void WPISwerveDrive::Configure(SwerveConfig &config){
    m_ebrake = config.ebrake;
    m_maxDriveSpeed = config.maxDriveSpeed;
    m_maxTurnSpeed = config.maxTurnSpeed;
    m_orientation = config.orientation;
    m_frontLeftLocation = config.frontLeftLocation;
    m_frontRightLocation = config.frontRightLocation;
    m_backLeftLocation = config.backLeftLocation;
    m_backRightLocation = config.backRightLocation;
    SetIdleMode(config.idle_mode);
    //m_modules = config.modules;
    m_kinematics = new frc::SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    m_odometry = new frc::SwerveDriveOdometry<4>(m_kinematics, m_gyro->GetHeading(),
    wpi::array{m_modules[0]->GetPosition(), m_modules[1]->GetPosition(), m_modules[2]->GetPosition(), m_modules[3]->GetPosition()}, frc::Pose2d{});
    m_deadzone = config.deadzone;
    m_gyro = config.gyro;
    m_estimator = new frc::SwerveDrivePoseEstimator<4>(m_kinematics, m_gyro->GetHeading(), {}, frc::Pose2d(frc::Translation2d(), m_gyro->GetHeading()));
}


bool WPISwerveDrive::GetEbrake() {
    return m_ebrake;
}
void WPISwerveDrive::SetEbrake(bool ebrake) {
    m_ebrake = ebrake;
}
void WPISwerveDrive::Drive(double x_position, double y_position, double rotation) {
    x_position = ApplyDeadzone(x_position);
    y_position = ApplyDeadzone(y_position);
    
     Drive(
     (units::feet_per_second_t)x_position * m_maxDriveSpeed, 
     (units::feet_per_second_t)y_position * m_maxDriveSpeed, 
     (units::degrees_per_second_t)rotation * m_maxTurnSpeed);
}

void WPISwerveDrive::Drive(units::feet_per_second_t vx, units::feet_per_second_t vy, units::degrees_per_second_t omega) {

    if (!m_orientation)
    {
        Drive(frc::ChassisSpeeds{vx, vy, omega});   
    }
    else
    {
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, m_gyro->GetHeading());
        Drive(speeds);
    }
}

void WPISwerveDrive::Drive(frc::ChassisSpeeds speed) {
    
    // states = m_kinematics.ToSwerveModuleStates(speed);
    auto states = m_kinematics->ToSwerveModuleStates(speed);
    
    m_kinematics->DesaturateWheelSpeeds(&states, units::feet_per_second_t(m_maxDriveSpeed));


    std::vector<frc::SwerveModuleState> stateN;
    stateN.push_back(states[0]);
    stateN.push_back(states[1]);
    stateN.push_back(states[2]);
    stateN.push_back(states[3]);

    Drive(stateN);

}
void WPISwerveDrive::Drive(std::vector<frc::SwerveModuleState> &state) {
    if (!m_ebrake) {
        for(int i = 0; i < state.size(); i++){
            m_modules[i]->SetState(state[i]);
        }
    }

    m_estimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(), m_gyro->GetHeading(), {});
} 

bool WPISwerveDrive::GetIdleMode() {
    return false;
}

void WPISwerveDrive::SetIdleMode(bool idle_mode) {
     for(int i = 0; i < m_modules.size(); i++){

        m_modules[i]->SetIdleMode(idle_mode);

    }
}
void WPISwerveDrive::SetRobotOriented() {
    m_orientation = false;
}
void WPISwerveDrive::SetFieldOriented() {
    m_orientation = true; 
}
bool WPISwerveDrive::GetOrientedMode() {
    return m_orientation;
}

double WPISwerveDrive::ApplyDeadzone(double input)
{
    double output = 0;

    if (input > m_deadzone)
    {
        output = (input - m_deadzone) / (1 - m_deadzone);
    }
    else if (input < -(m_deadzone))
    {
        output = (input + m_deadzone) / (1 - m_deadzone);
    }

    return output;
}