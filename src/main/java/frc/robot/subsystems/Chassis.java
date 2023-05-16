// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  
  private final WPITalonFX m_MFL; //motor-front-left
  private final WPITalonFX m_MFR; 
  private final WPITalonFX m_MBL;
  private final WPITalonFX m_MBR;
  
  public Chassis() {
    m_MFL = new WPITalonFX(Constants.Chassis.CAN.MFL);
    m_MFR = new WPITalonFX(Constants.Chassis.CAN.MFR);
    m_MBL = new WPITalonFX(Constants.Chassis.CAN.MBL);
    m_MBR = new WPITalonFX(Constants.Chassis.CAN.MBR);
    
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
  }

 
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
