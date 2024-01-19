#include "AbsoluteEncoder.h"

void AbsoluteEncoder::Configure(AbsoluteEncoderConfig &config)
{
    m_encoder = config.encoder;
    m_is_inverted = config.is_inverted;

    m_configure = ctre::phoenix::sensors::CANCoderConfiguration();

    m_configure.sensorDirection = config.is_inverted;
    m_configure.magnetOffsetDegrees = static_cast<double>(config.zero_heading.Degrees());


    m_encoder->ConfigAllSettings(m_configure);
}

frc::Rotation2d AbsoluteEncoder::GetHeading()
{
    return frc::Rotation2d(static_cast<units::degree_t>(m_encoder->GetPosition()));
}

frc::Rotation2d AbsoluteEncoder::GetRawHeading()
{ 
    return frc::Rotation2d(static_cast<units::degree_t>(m_encoder->GetAbsolutePosition()));
}

bool AbsoluteEncoder::GetInverted()
{
    return m_is_inverted;
}

void AbsoluteEncoder::SetZeroHeading(frc::Rotation2d zero_heading) {

    m_configure.magnetOffsetDegrees = static_cast<double>(zero_heading.Degrees());
    m_encoder->ConfigAllSettings(m_configure);

    
}

void AbsoluteEncoder::SetInverted(bool inverted) {

    m_configure.sensorDirection = inverted;

    m_encoder->ConfigAllSettings(m_configure);

}
