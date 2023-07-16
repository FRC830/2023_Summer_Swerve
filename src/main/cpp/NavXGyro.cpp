#include "NavXGyro.h"


void NavXGyro::Configure(GyroConfig &config)
{
    m_gyro = new AHRS(frc::SPI::Port::kMXP);
}

frc::Rotation3d NavXGyro::GetYawPitchRoll() 
{
    units::radian_t yaw = units::radian_t(m_gyro->GetYaw());
    units::radian_t pitch = units::radian_t(m_gyro->GetPitch());
    units::radian_t roll = units::radian_t(m_gyro->GetRoll());
    frc::Rotation3d yaw_pitch_roll = frc::Rotation3d(roll, pitch, yaw);
    return yaw_pitch_roll;
}

frc::Rotation2d NavXGyro::GetHeading() 
{
    return frc::Rotation2d();
}

frc::Rotation2d NavXGyro::GetRawHeading() 
{
    return frc::Rotation2d();
}

bool NavXGyro::GetInverted() 
{
    return false;
}

void NavXGyro::SetInverted(bool inverted) 
{

}

void NavXGyro::SetZeroHeading(double zero_heading) 
{

}
