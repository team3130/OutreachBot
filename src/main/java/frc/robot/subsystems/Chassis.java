// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Consumer;

public class Chassis extends SubsystemBase {
  private final WPI_TalonSRX m_leftFront;
  private final WPI_TalonSRX m_rightFront;
  private final WPI_TalonSRX m_leftBack;
  private final WPI_TalonSRX m_rightBack;

  private final MotorControllerGroup m_right;
  private final MotorControllerGroup m_left;

  private final DifferentialDrive m_drive;
  private final CANCoder m_leftAbsoluteEncoder; // the can encoder attached to the shaft
  private final CANCoder m_rightAbsoluteEncoder;

  public Chassis() {
    m_leftFront = new WPI_TalonSRX(Constants.CAN.leftFrontDrivetrain);
    m_rightFront = new WPI_TalonSRX(Constants.CAN.rightFrontDrivetrain);
    m_leftBack = new WPI_TalonSRX(Constants.CAN.leftBackDrivetrain);
    m_rightBack = new WPI_TalonSRX(Constants.CAN.rightBackDrivetrain);

    m_leftAbsoluteEncoder = new CANCoder(Constants.CAN.drivetrainLeftEncoder);
    m_rightAbsoluteEncoder = new CANCoder(Constants.CAN.drivetrainRightEncoder);

    m_rightFront.configFactoryDefault();
    m_leftFront.configFactoryDefault();
    m_rightBack.configFactoryDefault();
    m_leftBack.configFactoryDefault();

    m_rightFront.configVoltageCompSaturation(Constants.Drivetrain.maxVoltage);
    m_leftFront.configVoltageCompSaturation(Constants.Drivetrain.maxVoltage);
    m_rightBack.configVoltageCompSaturation(Constants.Drivetrain.maxVoltage);
    m_leftBack.configVoltageCompSaturation(Constants.Drivetrain.maxVoltage);
    m_rightFront.enableVoltageCompensation(true);
    m_leftFront.enableVoltageCompensation(true);
    m_rightBack.enableVoltageCompensation(true);
    m_leftBack.enableVoltageCompensation(true);

    m_rightFront.setInverted(true);
    m_rightBack.setInverted(true);

    configureBreakMode(true);

    m_right = new MotorControllerGroup(m_rightFront, m_rightBack);
    m_left = new MotorControllerGroup(m_leftFront, m_leftBack);

    m_drive = new DifferentialDrive(m_left, m_right);
    m_drive.setDeadband(Constants.Drivetrain.inputDeadband);
    m_drive.setSafetyEnabled(false);

    m_leftBack.follow(m_leftFront); //TODO ??
    m_rightBack.follow(m_rightFront); //TODO ??
    circleFixer = (Double[] angle) -> {
      angle[0] = ((angle[0] % 360) + 360) % 360;
      angle[0] += ((angle[0] > 180) ? -360 : 0);
    };




  }
  public void configureBreakMode(boolean brake) {
    if (brake) {
      m_leftFront.setNeutralMode(NeutralMode.Brake);
      m_rightFront.setNeutralMode(NeutralMode.Brake);
      m_leftBack.setNeutralMode(NeutralMode.Brake);
      m_rightBack.setNeutralMode(NeutralMode.Brake);
    } else {
      m_leftFront.setNeutralMode(NeutralMode.Coast);
      m_rightFront.setNeutralMode(NeutralMode.Coast);
      m_leftBack.setNeutralMode(NeutralMode.Coast);
      m_rightBack.setNeutralMode(NeutralMode.Coast);
    }
  }
  public void driveArcade(double moveThrottle, double turnThrottle, boolean squaredInputs) {
    m_drive.arcadeDrive(moveThrottle, turnThrottle, squaredInputs);
  }
  private final Consumer<Double[]> circleFixer;

  public void configRampRate() {
    m_rightFront.configOpenloopRamp(Constants.Drivetrain.rampRate);
    m_leftFront.configOpenloopRamp(Constants.Drivetrain.rampRate);
    m_rightBack.configOpenloopRamp(Constants.Drivetrain.rampRate);
    m_leftBack.configOpenloopRamp(Constants.Drivetrain.rampRate);
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
