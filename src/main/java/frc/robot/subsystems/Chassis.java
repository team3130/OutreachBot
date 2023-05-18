// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  
  private final WPI_TalonFX m_MFL; //motor-front-left
  private final WPI_TalonFX m_MFR; 
  private final WPI_TalonFX m_MBL;
  private final WPI_TalonFX m_MBR;
  
  public Chassis() {
    m_MFL = new WPI_TalonFX(Constants.CAN.MFL);
    m_MFR = new WPI_TalonFX(Constants.CAN.MFR);
    m_MBL = new WPI_TalonFX(Constants.CAN.MBL);
    m_MBR = new WPI_TalonFX(Constants.CAN.MBR);
    
    m_MFL.configFactoryDefault();
    m_MFR.configFactoryDefault();
    m_MBL.configFactoryDefault();
    m_MBR.configFactoryDefault();
    
    m_MFL.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_MFR.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_MBL.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_MBR.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
      
    m_MFL.enableVoltageCompensation(true);
    m_MFR.enableVoltageCompensation(true);
    m_MBL.enableVoltageCompensation(true);
    m_MBR.enableVoltageCompensation(true);
    
    m_MFL.setInverted(true);
    m_MFR.setInverted(true);
    m_MBL.setInverted(true);
    m_MBR.setInverted(true);
  }
 
  
  
  public double getMotorVelocity(WPI_TalonFX motor){
    return (motor.getSelectedSensorVelocity() / Constants.Chassis.encoderResolution
                * (Constants.Chassis.gearRatio) * (Math.PI * Constants.Chassis.wheelDiameter)) * 10;
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
