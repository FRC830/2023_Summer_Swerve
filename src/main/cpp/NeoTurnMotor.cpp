#include"NeoTurnMotor.h"


void NeoTurnMotor::Configure(SwerveTurnMotorConfig &config){
    m_AbsouluteEncoder = config.absouluteEncoder;
    m_turn_motor = config.turn_motor;
    m_relative_Encoder = config.relative_Encoder;
    m_PID = config.PID;
    m_turn_motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    
    m_relative_Encoder->SetPositionConversionFactor(config.ratio);
    SetInverted(config.inverted);
    m_turn_motor->BurnFlash();
};    

void NeoTurnMotor::SetRotation(frc::Rotation2d deg){
    frc::Rotation2d realTurn = deg - GetRotation();
    if(realTurn.Degrees().to<double>() > 180.0) {

        realTurn = realTurn - frc::Rotation2d(units::degree_t(360.0));

    }
   
    double targetPos = m_AbsouluteEncoder->GetHeading().Degrees().to<double>() + realTurn.Degrees().to<double>();
    m_PID->SetReference(targetPos, rev::CANSparkMax::ControlType::kPosition);
}; 

frc::Rotation2d NeoTurnMotor::GetRotation(){
    return m_AbsouluteEncoder->GetHeading();
}; 

bool NeoTurnMotor::GetInverted(){
    return m_turn_motor->GetInverted();
}; 
void NeoTurnMotor::SetInverted(bool invert){
    m_turn_motor->SetInverted(invert);
}; 
void NeoTurnMotor::ForceTurnDirectionCW(){
};
void NeoTurnMotor::ForceTurnDirectionCCW(){
};

//12.8:1
//537.6 
