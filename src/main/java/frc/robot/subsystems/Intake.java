// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private final WPI_TalonSRX m_motor;
  private double speed = 0.5;

  public Intake() {
    m_motor = new WPI_TalonSRX(Constants.CAN.Intake_Motor);
    m_motor.setInverted(false);
    m_motor.configVoltageCompSaturation(9.5);
    m_motor.enableVoltageCompensation(true);
  }

  public void spinIntake() {
    m_motor.set(speed);
  }

  public void spoutTake() {
    m_motor.set(-speed);
  }

  public double getSpeed() {
    return speed;
  }

  public void setSpeed(double newSpeed) {
    speed = newSpeed;
  }

  public void stop() {
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Intake speed", this::getSpeed, this::setSpeed);
  }

}
