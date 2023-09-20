// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX m_Intake;

  public Intake() {
    m_Intake = new WPI_TalonSRX(Constants.CAN.Intake_Motor);
    m_Intake.setInverted(false);
    m_Intake.configFactoryDefault();
  }

  public void Spintake() {
    m_Intake.set(ControlMode.PercentOutput, 0.7);
  }

  public void Stoptake() {
    m_Intake.set(ControlMode.PercentOutput, 0);
  }

  public void Spoutake() {
    m_Intake.set(ControlMode.PercentOutput, -0.7);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
