// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX flywheel; //motor that actually shoots the balls
  private final WPI_TalonSRX leftIndexer; //left side motor that pushes balls into the shooter
  private final WPI_VictorSPX rightIndexer; //left side motor that pushes balls into the shooter
  private double indexerSpeed = 0.4; //default 40% output for both indexer wheels
  private double flywheelSpeed = 0.7; //default 70% output for the flywheel
  private double delayTime = 0.15; //delay time between flywheel start up and indexer pushing balls into the shooter


  public Shooter() {
    //instantiate all motors
    flywheel = new WPI_TalonSRX(Constants.CAN.SHOOTERFLYWHEEL);
    leftIndexer = new WPI_TalonSRX(Constants.CAN.LEFTINDEXER);
    rightIndexer = new WPI_VictorSPX(Constants.CAN.RIGHTINDEXER);

    //configuring factory default and setting whether or not something is inverted aka direction
    flywheel.configFactoryDefault();
    flywheel.setInverted(true);
    leftIndexer.configFactoryDefault();
    leftIndexer.setInverted(false);
    rightIndexer.configFactoryDefault();
    rightIndexer.setInverted(false);
  }

  /** GENERAL METHODS */

  public void runFlywheel() {  //run flywheel at a given output
    flywheel.set(ControlMode.PercentOutput, flywheelSpeed);
  }
  public void reverseFlywheel() {  //run flywheel at a given percent output in reverse to empty it
    flywheel.set(ControlMode.PercentOutput, -flywheelSpeed);
  }

  public void spinIndexer() { //spin indexer at a given percent output to feed flywheel
    leftIndexer.set(ControlMode.PercentOutput, indexerSpeed);
    rightIndexer.set(ControlMode.PercentOutput, indexerSpeed);
  }

  public void reverseIndexer() { //spin indexer at a given percent output back towards hopper to clear congestion
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

  /** GETTERS AND SETTERS */
  public void setIndexerSpeed(double newSpeed){ //set or update the variable indexerSpeed to the value newSpeed
    indexerSpeed = newSpeed;
  }
  public double getIndexerSpeed(){ //get or return the variable indexerSpeed
    return indexerSpeed;
  }

  public void setFlywheelSpeed(double newSpeed){ //set or update the variable flywheelSpeed to the value newSpeed
    flywheelSpeed = newSpeed;
  }
  public double getFlywheelSpeed(){ //get or return the variable flywheelSpeed
    return flywheelSpeed;
  }

  public double getDelayTime(){ //get or return the variable delayTime
    return delayTime;
  }
  public void setDelayTime(double delay){ //set or update the variable delayTime to the value delay
    delayTime = delay;
  }


  @Override
  public void periodic() {
    // This built-in method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) { // outputs to shuffleboard in a way that can be update-able in real time (many of these can be removed after testing)
    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("flywheel speed", this::getFlywheelSpeed, this::setFlywheelSpeed);
    builder.addDoubleProperty("indexer speed", this::getIndexerSpeed, this::setIndexerSpeed);
    builder.addDoubleProperty("wait time", this::getDelayTime, this::setDelayTime);
    //builder.addVariableTypeProperty("name to display", this:getter, if you want it to be editable-> this::setter else -> null);
  }

  @Override
  public void simulationPeriodic() {
    // This built-in method will be called once per scheduler run during simulation
  }
}
