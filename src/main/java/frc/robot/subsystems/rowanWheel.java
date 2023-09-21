// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class rowanWheel extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX m_motor;


  public rowanWheel() {
    m_motor = new WPI_TalonSRX(Constants.CANID.Intake_Motor);
    m_intake.setInverted(true);
    m_motor.configFactoryDefault();
  }
}


public void runMotor(){
  m_motor.set(ControlMode.PercentOutput, 0.5);
}


public void stopMotor(){
  m_motor.set(ControlMode.PercentOutput, 0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


