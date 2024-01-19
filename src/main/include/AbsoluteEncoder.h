#pragma once
#include "Interfaces/SwerveAbsoluteEncoder.h"
#include <CTRE/phoenix/sensors/CANCoder.h>

struct AbsoluteEncoderConfig
{
    ctre::phoenix::sensors::CANCoder *encoder;
    bool is_inverted;
    frc::Rotation2d zero_heading;
};

class AbsoluteEncoder : public SwerveAbsoluteEncoder
{
    public:
        AbsoluteEncoder() = default;
        virtual ~AbsoluteEncoder() = default;
        virtual void Configure(AbsoluteEncoderConfig &config) override;
        virtual frc::Rotation2d GetHeading() override;
        virtual frc::Rotation2d GetRawHeading() override;
        virtual bool GetInverted() override;
        virtual void SetInverted(bool inverted) override;
        virtual void SetZeroHeading(frc::Rotation2d zero_heading) override;

        
    private:
        ctre::phoenix::sensors::CANCoder *m_encoder;
        bool m_is_inverted;
        ctre::phoenix::sensors::CANCoderConfiguration m_configure;
        frc::Rotation2d m_zero_heading;
};