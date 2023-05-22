// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX flywheel;
  private final WPI_TalonSRX leftIndexer;
  private final WPI_TalonSRX rightIndexer;
  private final SimpleMotorFeedforward flywheelFeedForward;
  private final PIDController flywheelPID;
  private double flywheelF = 0;
  private double flywheelP = 0;
  private double flywheelI = 0;
  private double flywheelD = 0;
  private double rpm = 100d;

  //private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  /*

  private final GenericEntry flywheelRPMSetpoint = tab.add("Flywheel RPM", rpm).getEntry();
  private final GenericEntry currentRPM = tab.add("Flywheel Current RPM", 0).getEntry();

  private final GenericEntry P = tab.add("Top Flywheel P", flywheelP).getEntry();
  private final GenericEntry I = tab.add("Top Flywheel I", flywheelI).getEntry();
  private final GenericEntry D = tab.add("Top Flywheel D", flywheelD).getEntry();
  private final GenericEntry V = tab.add("Top Flywheel V", flywheelF).getEntry();

*/

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

    flywheelFeedForward = new SimpleMotorFeedforward(0, flywheelF);
    flywheelPID = new PIDController(flywheelP, flywheelI, flywheelD);

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

  public void runFlywheelControlled(double current, double setpoint){
    flywheel.setVoltage(flywheelFeedForward.calculate(current, setpoint) + flywheelPID.calculate(current, setpoint));
  }


  public double getFlywheelP() {
    return flywheelP;
  }

  public void setFlywheelP(double flywheelP) {
    this.flywheelP = flywheelP;
  }

  public double getFlywheelI() {
    return flywheelI;
  }

  public void setFlywheelI(double flywheelI) {
    this.flywheelI = flywheelI;
  }

  public double getFlywheelD() {
    return flywheelD;
  }

  public void setFlywheelD(double flywheelD) {
    this.flywheelD = flywheelD;
  }

  public double getRpmSetpoint() {
    return rpm;
  }

  public void setRpmSetpoint(double rpm) {
    this.rpm = rpm;
  }

  public double getCurrentFlywheelNative(){
    return flywheel.getSelectedSensorVelocity();
  }
  public double getFlywheelF(){
    return flywheelF;
  }
  public void setFlywheelF( double f){
    this.flywheelF = f;
  }

  public double getRPMSetpointInNative(){
    return this.getRpmSetpoint() * Constants.Shooter.FlywheelRPMtoNativeUnitsScalar;
  }
  public double getCurrentFlywheelInRPM(){
    return this.getCurrentFlywheelNative() / Constants.Shooter.FlywheelRPMtoNativeUnitsScalar;
  }
/*
  public void outputToShuffleboard(){
    SmartDashboard.putNumber("Flywheel P", flywheelP);
    SmartDashboard.putNumber("Flywheel I", flywheelI);
    SmartDashboard.putNumber("Flywheel D", flywheelD);
    SmartDashboard.putNumber("Flywheel F", flywheelF);
    SmartDashboard.putNumber("Flywheel RPM", rpm);
    }
    */

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("Flywheel P", this::getFlywheelP, this::setFlywheelP);
    builder.addDoubleProperty("Flywheel I", this::getFlywheelI, this::setFlywheelI);
    builder.addDoubleProperty("Flywheel D", this::getFlywheelD, this::setFlywheelD);
    builder.addDoubleProperty("Flywheel F", this::getFlywheelF, this::setFlywheelF);
    builder.addDoubleProperty("Flywheel RPM", this::getRpmSetpoint, this::setRpmSetpoint);
    builder.addDoubleProperty("Current Flywheel RPM", this::getCurrentFlywheelInRPM, null);
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
