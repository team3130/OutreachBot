// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX flywheel;
  private final WPI_TalonSRX leftIndexer;
  private final WPI_TalonSRX rightIndexer;
  private double indexerSpeed = 0.4;
  private double flywheelSpeed = 0.8;
  private double delayTime = 0.2;


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

  /** miscellaneous methods */
  public void runFlywheel() {  //run flywheel at a given output
    flywheel.set(ControlMode.PercentOutput, flywheelSpeed);
  }

  public void spinIndexer() { //spin indexer to feed flywheel
    leftIndexer.set(ControlMode.PercentOutput, indexerSpeed);
    rightIndexer.set(ControlMode.PercentOutput, indexerSpeed);
  }

  public void reverseIndexer() { //spin indexer back towards hopper to clear congestion
    leftIndexer.set(ControlMode.PercentOutput, -indexerSpeed);
    rightIndexer.set(ControlMode.PercentOutput, -indexerSpeed);
  }
  public void stopFlywheel() { //stop flywheel motor
    flywheel.stopMotor();
  }

  public void stopIndexer() { //stop both indexer motors
    leftIndexer.stopMotor();
    rightIndexer.stopMotor();
  }

  /**getters & setters */
  public void setIndexerSpeed(double newSpeed){
    indexerSpeed = newSpeed;
  }
  public double getIndexerSpeed(){
    return indexerSpeed;
  }

  public void setFlywheelSpeed(double newSpeed){
    flywheelSpeed = newSpeed;
  }
  public double getFlywheelSpeed(){
    return flywheelSpeed;
  }

  public double getDelayTime(){
    return delayTime;
  }
  public void setDelayTime(double delay){
    delayTime = delay;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("flywheel speed", this::getFlywheelSpeed, this::setFlywheelSpeed);
    builder.addDoubleProperty("indexer speed", this::getIndexerSpeed, this::setIndexerSpeed);
    builder.addDoubleProperty("wait time", this::getDelayTime, this::setDelayTime);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
