// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private final WPI_TalonFX m_motor;

  public Intake() {
    m_motor = new WPI_TalonFX(Constants.Intake.CAN_Intake_Motor);

    m_motor.setInverted(false);

    m_motor.configVoltageCompSaturation(9.5);
    m_motor.enableVoltageCompensation(true);
    
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public void disable() {
    m_motor.set(0);
  }

  public void toggleVoltageCompensation() {
    m_motor.enableVoltageCompensation(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
