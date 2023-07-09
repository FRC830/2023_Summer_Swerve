#include "WPISwerveDrive.h"

void WPISwerveDrive::Configure(SwerveConfig &config){
    SetIdleMode(config.idle_mode);
    m_ebrake = config.ebrake;
    m_maxDriveSpeed = config.maxDriveSpeed;
    m_maxTurnSpeed = config.maxTurnSpeed;
    m_orientation = config.orientation;
    

}


bool WPISwerveDrive::GetEbrake() {
    return m_ebrake;
}
void WPISwerveDrive::SetEbrake(bool ebrake) {
    m_ebrake = ebrake;
}
void WPISwerveDrive::Drive(double x_position, double y_position, double rotation) {
    

    Drive(x_position * m_maxDriveSpeed, y_position * m_maxDriveSpeed, rotation * m_maxTurnSpeed);
}
void WPISwerveDrive::Drive(double vx, double vy, double omega) {
    
    Drive(frc::ChassisSpeeds((units::meters_per_second_t)vx, (units::meters_per_second_t)vy, (units::radians_per_second_t)omega));

}
void WPISwerveDrive::Drive(frc::ChassisSpeeds speed) {

    frc::SwerveDriveKinematics<4> m_kinematics{
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
    m_backRightLocation};
 

    auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speed);
    
    states.push_back(fl);
    states.push_back(fr);
    states.push_back(bl);
    states.push_back(br);

    Drive(states);

}
void WPISwerveDrive::Drive(std::vector<frc::SwerveModuleState> &state) {

} 
bool WPISwerveDrive::GetIdleMode() {

}
void WPISwerveDrive::SetIdleMode(bool idle_mode) {

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