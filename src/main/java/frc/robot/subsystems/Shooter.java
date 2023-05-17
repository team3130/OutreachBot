// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX flywheel;
  private final WPI_TalonSRX leftIndexer;
  private final WPI_TalonSRX rightIndexer;

  public Shooter() {
    flywheel = new WPI_TalonSRX(Constants.CAN.SHOOTERFLYWHEEL);
    leftIndexer = new WPI_TalonSRX(Constants.CAN.LEFTINDEXER);
    rightIndexer = new WPI_TalonSRX(Constants.CAN.RIGHTINDEXER);

    flywheel.configFactoryDefault();
    flywheel.setInverted(false);
    leftIndexer.configFactoryDefault();
    leftIndexer.setInverted(false);
    rightIndexer.configFactoryDefault();
    rightIndexer.setInverted(false);
  }

  public void runFlywheel() {
    flywheel.set(ControlMode.PercentOutput, 0.75);
  }

  public void spinIndexer() {
    leftIndexer.set(ControlMode.PercentOutput, 0.4);
    rightIndexer.set(ControlMode.PercentOutput, 0.4);
  }

  public void reverseIndexer() {
    leftIndexer.set(ControlMode.PercentOutput, -0.4);
    rightIndexer.set(ControlMode.PercentOutput, -0.4);
  }

  public void stopFlywheel() {
    flywheel.stopMotor();
  }

  public void stopIndexer() {
    leftIndexer.stopMotor();
    rightIndexer.stopMotor();
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
