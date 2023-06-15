// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private final WPI_TalonSRX m_motor;
  private double speed = 0.9;
  private double volt = 9.5;
  private boolean voltComp = true;

  public Intake() {
    m_motor = new WPI_TalonSRX(Constants.CAN.Intake_Motor);
    m_motor.setInverted(true);
    m_motor.configFactoryDefault();
    m_motor.configVoltageCompSaturation(volt);
    m_motor.enableVoltageCompensation(voltComp);
  }

  /** miscellaneous methods */
  public void spinIntake() { //spin beater bar to intake
    m_motor.set(ControlMode.PercentOutput, speed);
  }
  public void spoutTake() { //spin beater bar to eject out of intake
    m_motor.set(ControlMode.PercentOutput, -speed);
  }
  public void updateEnableVoltageCompensation(boolean bool){ //update if voltage compensation is enabled from shuffleboard
    m_motor.enableVoltageCompensation(bool);
  }
  public void updateVoltageCompensationNum(double vol){ //update the voltage threshold from shuffleboard
    m_motor.configVoltageCompSaturation(vol);
  }
  public void stop() { //stop motor
    m_motor.set(ControlMode.PercentOutput, 0);
  }

  /** getters & setters */
  public double getSpeed() {
    return speed;
  }
  public void setSpeed(double newSpeed) {
    speed = newSpeed;
  }

  public boolean getVoltageCompBoolean(){
    return voltComp;
  }
  public void setVoltageCompBoolean(boolean bool){
    if (voltComp != bool){
      voltComp = !voltComp;
    }
  }

  public double getVoltNum(){
    return volt;
  }
  public void setVoltNum(double newVolt){
    volt = newVolt;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override // outputs to shuffleboard in a way that is update-able in real time (many of these can be removed after testing)
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Intake speed", this::getSpeed, this::setSpeed);
    builder.addDoubleProperty("Voltage compensation double", this::getVoltNum, this::setVoltNum);
    builder.addBooleanProperty("Voltage compensation boolean", this::getVoltageCompBoolean, this::setVoltageCompBoolean);
  }

}
