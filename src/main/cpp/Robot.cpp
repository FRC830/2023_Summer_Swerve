// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::SwerveInit(){
  AbsoluteEncoderConfig brAbsConfig;
  brAbsConfig.encoder = &br_abs_enc;
  brAbsConfig.is_inverted = BR_ABS_ENC_INVERTED;
  brAbsConfig.zero_heading = BR_ZERO_HEADING;
  AbsoluteEncoderConfig frAbsConfig;
  frAbsConfig.encoder = &fr_abs_enc;
  frAbsConfig.is_inverted = BR_ABS_ENC_INVERTED;
  frAbsConfig.zero_heading = BR_ZERO_HEADING;
  AbsoluteEncoderConfig blAbsConfig;
  AbsoluteEncoderConfig flAbsConfig;

  AnalogAbsoluteEncoder brAbs;
  brAbs.Configure(brAbsConfig);
  AnalogAbsoluteEncoder frAbs;
  frAbs.Configure(frAbsConfig);



  SwerveTurnMotorConfig brTurnConfig;
  brTurnConfig.absouluteEncoder=&brAbs;
  brTurnConfig.d=TURN_D;
  brTurnConfig.deviceID=BR_TURN_MTR_ID;
  brTurnConfig.ff=TURN_FF;
  brTurnConfig.i=TURN_I;
  brTurnConfig.inverted=BR_TURN_MTR_INVERTED;
  brTurnConfig.p=TURN_P;
  brTurnConfig.PID=&br_turn_pid;
  brTurnConfig.ratio=TURN_GEAR_RATIO;
  brTurnConfig.relative_Encoder=&br_turn_enc;
  brTurnConfig.turn_motor=&br_turn_mtr;
  SwerveTurnMotorConfig frTurnConfig;
  frTurnConfig.absouluteEncoder=&frAbs;
  frTurnConfig.d=TURN_D;
  frTurnConfig.deviceID=BR_TURN_MTR_ID;
  frTurnConfig.ff=TURN_FF;
  frTurnConfig.i=TURN_I;
  frTurnConfig.inverted=BR_TURN_MTR_INVERTED;
  frTurnConfig.p=TURN_P;
  frTurnConfig.PID=&fr_turn_pid;
  frTurnConfig.ratio=TURN_GEAR_RATIO;
  frTurnConfig.relative_Encoder=&fr_turn_enc;
  frTurnConfig.turn_motor=&fr_turn_mtr;
  SwerveTurnMotorConfig blTurnConfig;
  SwerveTurnMotorConfig flTurnConfig;

  NeoTurnMotor brTurn;
  brTurn.Configure(brTurnConfig);
  NeoTurnMotor frTurn;
  frTurn.Configure(frTurnConfig);


  SwerveDriveMotorConfig brDriveConfig;
  brDriveConfig.d = DRIVE_D;
  brDriveConfig.ff = DRIVE_FF;
  brDriveConfig.i = DRIVE_I;
  brDriveConfig.p = DRIVE_P;
  brDriveConfig.PID = &br_drive_pid;
  brDriveConfig.ratio = MOTOR_ROT_TO_FT / 60.0;
  brDriveConfig.encoder = &br_drive_enc;
  brDriveConfig.motor = &br_drive_mtr;
  SwerveDriveMotorConfig frDriveConfig;
  frDriveConfig.d = DRIVE_D;
  frDriveConfig.ff = DRIVE_FF;
  frDriveConfig.i = DRIVE_I;
  frDriveConfig.p = DRIVE_P;
  frDriveConfig.PID = &fr_drive_pid;
  frDriveConfig.ratio = MOTOR_ROT_TO_FT / 60.0;
  frDriveConfig.encoder = &fr_drive_enc;
  frDriveConfig.motor = &fr_drive_mtr;
  SwerveDriveMotorConfig blDriveConfig;
  SwerveDriveMotorConfig flDriveConfig;

  NeoDriveMotor brDrive;
  brDrive.Configure(brDriveConfig);
  NeoDriveMotor frDrive;
  frDrive.Configure(frDriveConfig);

  SwerveModuleConfig brModuleConfig;
  brModuleConfig.driveMotor=&brDrive;
  brModuleConfig.idleMode=true;
  brModuleConfig.turnMotor=&brTurn;
  SwerveModuleConfig frModuleConfig;
  frModuleConfig.driveMotor=&frDrive;
  frModuleConfig.idleMode=true;
  frModuleConfig.turnMotor=&frTurn;
  SwerveModuleConfig blModuleConfig;
  SwerveModuleConfig flModuleConfig;

  SwerveConfig swerveConfig;
  swerveConfig.backLeftLocation=bl_position;
  swerveConfig.backRightLocation=br_position;
  swerveConfig.frontLeftLocation=fl_position;
  swerveConfig.frontRightLocation=fr_position;
  swerveConfig.ebrake=true;
  swerveConfig.idle_mode=true;
  swerveConfig.maxDriveSpeed=MAX_DRIVE_SPEED_MPS;
  swerveConfig.maxTurnSpeed=MAX_ANGULAR_VELOCITY_DEGPS;
  swerveConfig.orientation=IS_ROBOT_ORIENTED_DRIVE;
  m_swerveDrive.Configure(swerveConfig);


}

void Robot::RobotInit() {
  SwerveInit();
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() 
{
  /* AbsoluteEncoderConfig config;
  config.encoder = &bl_abs_enc;
  config.is_inverted = true;
  m_back_left_analog_encoder.Configure(config);
  m_back_left_analog_encoder.SetZeroHeading(m_back_left_analog_encoder.GetRawHeading());

  SwerveTurnMotorConfig nConfig;
  nConfig.absouluteEncoder= &m_back_left_analog_encoder;
  nConfig.deviceID = 4;
  nConfig.inverted = true;
  nConfig.turn_motor = &bl_turn_mtr;
  nConfig.relative_Encoder = &bl_turn_enc;
  nConfig.PID = &bl_turn_pid; 
  nConfig.p = TURN_P;
  nConfig.i = TURN_I;
  nConfig.d = TURN_D;
  nConfig.ff = TURN_FF;
  nConfig.ratio = 1;
  m_back_left_turn_motor.Configure(nConfig); */

  //   bl_turn_pid.SetP(0.005);
  //   bl_turn_pid.SetI(0);
  //   bl_turn_pid.SetD(0.03);
  // bl_turn_enc.SetPositionConversionFactor(MOTOR_ROT_TO_DEG);
  // bl_turn_mtr.BurnFlash(); 
  // // Absolute Encoder configuration
  // AbsoluteEncoderConfig absolute_encoder_config;
  // absolute_encoder_config.is_inverted = true;
  // m_back_left_analog_encoder.Configure(absolute_encoder_config);
  // m_back_left_analog_encoder.SetZeroHeading(m_back_left_analog_encoder.GetRawHeading());

  // // NavX Gyro Configuration
  // GyroConfig gyro_config;
  // gyro_config.is_inverted = true;
  // gyro_config.zero_heading = units::degree_t(90);
  // gyro.Configure(gyro_config);
  AbsoluteEncoderConfig fl_encoderConfig;
  fl_encoderConfig.encoder = &fl_abs_enc;
  fl_encoderConfig.is_inverted = FL_ABS_ENC_INVERTED;
  fl_encoderConfig.zero_heading = FL_ZERO_HEADING;

  m_ABSencoder.Configure(fl_encoderConfig);
  m_ABSencoder.SetZeroHeading(m_ABSencoder.GetRawHeading());

  SwerveTurnMotorConfig fl_turnConfig;
  fl_turnConfig.absouluteEncoder = &m_ABSencoder;
  fl_turnConfig.d = TURN_D;
  fl_turnConfig.deviceID = BL_TURN_MTR_ID;
  fl_turnConfig.ff = TURN_FF;
  fl_turnConfig.i = TURN_I;
  fl_turnConfig.inverted = BL_TURN_MTR_INVERTED;
  fl_turnConfig.p = TURN_P;
  fl_turnConfig.PID = &bl_turn_pid;
  fl_turnConfig.ratio = MOTOR_ROT_TO_DEG;
  fl_turnConfig.relative_Encoder = &bl_turn_enc;
  fl_turnConfig.turn_motor = &bl_turn_mtr;

  SwerveDriveMotorConfig fl_driveConfig;
  fl_driveConfig.d = DRIVE_D;
  fl_driveConfig.ff = DRIVE_FF;
  fl_driveConfig.i = DRIVE_I;
  fl_driveConfig.p = DRIVE_P;
  fl_driveConfig.PID = &fl_drive_pid;
  fl_driveConfig.ratio = MOTOR_ROT_TO_FT / 60.0;
  fl_driveConfig.encoder = &fl_drive_enc;
  fl_driveConfig.motor = &fl_drive_mtr;

  m_driveMotor.Configure(fl_driveConfig);


  m_swerveModule_FL.Configure(moduleConfig);

  // AbsoluteEncoderConfig encoderConfig;
  // encoderConfig.encoder = &bl_abs_enc;
  // encoderConfig.is_inverted = BL_ABS_ENC_INVERTED;
  // encoderConfig.zero_heading = BL_ZERO_HEADING;

  // m_ABSencoder.Configure(encoderConfig);

  // m_ABSencoder.SetZeroHeading(m_ABSencoder.GetRawHeading());
  
  // SwerveTurnMotorConfig turnConfig;
  // turnConfig.absouluteEncoder = &m_ABSencoder;
  // turnConfig.d = TURN_D;
  // turnConfig.deviceID = BL_TURN_MTR_ID;
  // turnConfig.ff = TURN_FF;
  // turnConfig.i = TURN_I;
  // turnConfig.inverted = BL_TURN_MTR_INVERTED;
  // turnConfig.p = TURN_P;
  // turnConfig.PID = &bl_turn_pid;
  // turnConfig.ratio = MOTOR_ROT_TO_DEG;
  // turnConfig.relative_Encoder = &bl_turn_enc;
  // turnConfig.turn_motor = &bl_turn_mtr;

  // m_turnMotor.Configure(turnConfig);

  // SwerveDriveMotorConfig driveConfig;
  // driveConfig.d = DRIVE_D;
  // driveConfig.ff = DRIVE_FF;
  // driveConfig.i = DRIVE_I;
  // driveConfig.p = DRIVE_P;
  // driveConfig.PID = &bl_drive_pid;
  // driveConfig.ratio = MOTOR_ROT_TO_FT / 60.0;
  // driveConfig.encoder = &bl_drive_enc;
  // driveConfig.motor = &bl_drive_mtr;

  // m_driveMotor.Configure(driveConfig);

  
  // SwerveModuleConfig moduleConfig;
  // moduleConfig.driveMotor = &m_driveMotor;
  // moduleConfig.idleMode = true;
  // moduleConfig.turnMotor = &m_turnMotor;


  // m_swerveModule_BL.Configure(moduleConfig);



  // frc::SmartDashboard::PutNumber("Speed", 0.0);
  // frc::SmartDashboard::PutNumber("ang", 0.0);
  
  

  // 
}

void Robot::TeleopPeriodic() 
{
  // //m_back_left_turn_motor.SetRotation(frc::Rotation2d(static_cast<units::degree_t>(frc::SmartDashboard::GetNumber("backleft turn motor position", 69))));
  // //std::cout << "\rCurrent motor heading/positon:\t" << static_cast<double>(m_back_left_analog_encoder.GetHeading().Degrees()) << "\tsoftware motor positon" << frc::SmartDashboard::GetNumber("backleft turn motor position", 69);
  // double p = frc::SmartDashboard::GetNumber("p", 1);
  // double i = frc::SmartDashboard::GetNumber("i", 1);
  // double d = frc::SmartDashboard::GetNumber("d", 1);
  // double sp = frc::SmartDashboard::GetNumber("Set Point", 0);

  // if (p != bl_turn_pid.GetP() || i != bl_turn_pid.GetI() || d != bl_turn_pid.GetD())
  // {


  //   // frc::SmartDashboard::PutNumber("p", p);
  //   // frc::SmartDashboard::PutNumber("i", i);
  //   // frc::SmartDashboard::PutNumber("d", d);
  // }

  
  frc::SmartDashboard::PutNumber("Current Turn Position", bl_turn_enc.GetPosition());
  frc::SmartDashboard::PutNumber("Current Drive velocity", m_driveMotor.GetVelocity());
  frc::SmartDashboard::PutNumber("Current Absoluteheading", m_ABSencoder.GetHeading().Degrees().to<double>());

  auto speed = frc::SmartDashboard::GetNumber("Speed", 0.0);
  auto angle = frc::SmartDashboard::GetNumber("ang", 0.0);

  frc::SwerveModuleState state{units::feet_per_second_t(speed), units::degree_t(angle)};
  //bl_turn_pid.SetReference(angle, rev::CANSparkMax::ControlType::kPosition);
  //m_swerveModule_BL.SetState(state);

  
  // frc::SmartDashboard::PutNumber("Set Position", sp);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
