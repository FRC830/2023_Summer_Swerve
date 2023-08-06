#include "NeoDriveMotor.h"


NeoDriveMotor::NeoDriveMotor(){};
NeoDriveMotor::~NeoDriveMotor(){}
void NeoDriveMotor::Configure(SwerveDriveMotorConfig &config){
    m_encoder = config.encoder;
    m_motorID = config.motorID;
    m_motor = config.motor;
    m_PID = config.PID;
    m_motor->SetIdleMode(config.idleMode);
    m_encoder->SetVelocityConversionFactor(config.ratio);
    m_motor->BurnFlash();
    m_MaxSpeed = config.maxSpeed;
    
};

void NeoDriveMotor::SetVelocity(units::velocity::feet_per_second_t v) {


    
    m_PID->SetReference(v.to<double>(), rev::CANSparkMax::ControlType::kVelocity);

};


double NeoDriveMotor::GetVelocity() {
    return m_encoder->GetVelocity();

};



void NeoDriveMotor::SetIdleMode(bool m) {
    m_motor->SetIdleMode(m ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);

};


bool NeoDriveMotor::GetIdleMode() {
    return m_motor->GetIdleMode() == rev::CANSparkMax::IdleMode::kBrake;

};



