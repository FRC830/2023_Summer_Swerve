#include"NeoTurnMotor.h"

NeoTurnMotor::NeoTurnMotor(){};

NeoTurnMotor::~NeoTurnMotor(){};

 
void NeoTurnMotor::Configure(SwerveTurnMotorConfig &config){
    m_AbsouluteEncoder = config.absouluteEncoder;
    m_turn_motor = new rev::CANSparkMax(config.deviceID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    //m_relative_Encoder = &m_turn_motor->GetEncoder();
    //m_PID  = &m_turn_motor->GetPIDController();
    m_turn_motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    SetInverted(config.inverted);
    m_turn_motor->GetPIDController().SetP(1);
    m_turn_motor->GetPIDController().SetD(0.1);
    m_turn_motor->BurnFlash();
    

} ;
void NeoTurnMotor::SetRotation(frc::Rotation2d deg){
    frc::Rotation2d realTurn = deg - GetRotation();
    int turnTicks = static_cast<double>(realTurn.Degrees()) * ratio; 
    int currentTicks = m_turn_motor->GetEncoder().GetPosition() * 42; 
    m_turn_motor->GetEncoder().SetPosition(currentTicks+turnTicks);
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
