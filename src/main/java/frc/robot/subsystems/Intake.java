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
  
  private final WPI_TalonSRX m_intake; // motor attached to the beater bar to intake balls
  private double speed = 0.9; //default 90% output for the beater bar
  private double volt = 9.5; //voltage compensation max
  private boolean voltComp = true; //voltage compensation on

  public Intake() {
    //instantiate motor
    m_intake = new WPI_TalonSRX(Constants.CAN.Intake_Motor);

    //set whether its inverted / its direction
    m_intake.setInverted(true);

    //good practice to set to default settings
    m_intake.configFactoryDefault();

    //set voltage compensation to a number and turn it on
    m_intake.configVoltageCompSaturation(volt);
    m_intake.enableVoltageCompensation(voltComp);
  }

  /** GENERAL METHODS */
  public void spinIntake() { //spin beater bar to intake
    m_intake.set(ControlMode.PercentOutput, speed);
  }
  public void spoutTake() { //spin beater bar backwards to eject out of intake
    m_intake.set(ControlMode.PercentOutput, -speed); //negative percent output spins backward
  }
  public void updateEnableVoltageCompensation(boolean bool){ //update if voltage compensation is enabled from shuffleboard
    m_intake.enableVoltageCompensation(bool);
  }
  public void updateVoltageCompensationNum(double vol){ //update the voltage threshold from shuffleboard
    m_intake.configVoltageCompSaturation(vol);
  }
  public void stop() { //stop motor
    m_intake.set(ControlMode.PercentOutput, 0);
  }

  /** GETTERS AND SETTERS */
  public double getSpeed() { //gets or return the variable speed
    return speed;
  }
  public void setSpeed(double newSpeed) { //sets or updates the variable speed to the value newSpeed
    speed = newSpeed;
  }

  public boolean getVoltageCompBoolean(){ //gets or return the variable voltComp
    return voltComp;
  }
  public void setVoltageCompBoolean(boolean bool){ //sets or updates the variable voltComp to the value bool
    if (voltComp != bool){
      voltComp = !voltComp;
    }
  }

  public double getVoltNum(){ //gets or return the variable voltComp
    return volt;
  }
  public void setVoltNum(double newVolt){ //sets or updates the variable volt to the value newVolt
    volt = newVolt;
  }


  @Override
  public void periodic() {
    // This built-in method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This built-in method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) { // outputs to shuffleboard in a way that can be update-able in real time (many of these can be removed after testing)
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Intake speed", this::getSpeed, this::setSpeed);
    builder.addDoubleProperty("Voltage compensation double", this::getVoltNum, this::setVoltNum);
    builder.addBooleanProperty("Voltage compensation boolean", this::getVoltageCompBoolean, this::setVoltageCompBoolean);
    //builder.addVariableTypeProperty("name to display", this:getter, if you want it to be editable-> this::setter else -> null);
  }

}
