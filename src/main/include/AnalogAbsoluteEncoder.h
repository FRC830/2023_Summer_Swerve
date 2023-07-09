#pragma once
#include "SwerveAbsoluteEncoder.h"

class AnalogAbsoluteEncoder : public SwerveAbsoluteEncoder
{
    public:
        virtual void Configure(AbsoluteEncoderConfig &config) override;
        virtual frc::Rotation2d GetHeading() override;
        virtual frc::Rotation2d GetRawHeading() override;
        virtual bool GetInverted() override;
        virtual void SetInverted(bool inverted) override;
        virtual void SetZeroHeading(frc::Rotation2d zero_heading) override;
        
    private:

};