// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Navx;

public class Chassis extends SubsystemBase {
  
  private final WPI_TalonFX m_MFL; //motor-front-left
  private final WPI_TalonFX m_MBL;
  private final WPI_TalonFX m_MBR;

  private final WPI_VictorSPX m_MFR;

  private final DifferentialDrive m_drive;

  private final MotorControllerGroup m_motorsRight;
  private final MotorControllerGroup m_motorsLeft;

  private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDConttroller;

  private final Navx m_navx = Navx.GetInstance();

  
  public Chassis() {
    m_MFL = new WPI_TalonFX(Constants.CAN.frontLeftDrive);
    m_MFR = new WPI_VictorSPX(Constants.CAN.frontRightDrive);
    m_MBL = new WPI_TalonFX(Constants.CAN.backLeftDrive);
    m_MBR = new WPI_TalonFX(Constants.CAN.backRightDrive);
    
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

    m_MFR.setInverted(true);
    m_MBR.setInverted(true);

    m_motorsRight = new MotorControllerGroup(m_MFR, m_MBR);
    m_motorsLeft = new MotorControllerGroup(m_MFL, m_MBL);

    m_drive = new DifferentialDrive(m_motorsLeft, m_motorsRight);
    m_drive.setDeadband(Constants.Chassis.kDriveDeadband);
    m_drive.setSafetyEnabled(false);

    MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_MFR,m_MBR);
    MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_MFL,m_MBL);

    m_feedforward = new SimpleMotorFeedforward(Constants.Chassis.ChassiskS, Constants.Chassis.ChassiskV, Constants.Chassis.ChassiskA);
    m_leftPIDController = new PIDController(Constants.Chassis.LChassiskP, Constants.Chassis.LChassiskI, Constants.Chassis.LChassiskD);
    m_rightPIDConttroller = new PIDController(Constants.Chassis.RChassiskP, Constants.Chassis.RChassiskI, Constants.Chassis.RChassiskD);

    configureBrakeMode(true);
  }

  public void driveArcade(double moveThrottle, double turnThrottle, boolean squaredInputs) {
    m_drive.arcadeDrive(moveThrottle, turnThrottle, squaredInputs);
  }

  public void configureBrakeMode(boolean brake) {
    if (brake) {
      m_MFL.setNeutralMode(NeutralMode.Brake);
      m_MFR.setNeutralMode(NeutralMode.Brake);
      m_MBL.setNeutralMode(NeutralMode.Brake);
      m_MBR.setNeutralMode(NeutralMode.Brake);
    } else {
      m_MFL.setNeutralMode(NeutralMode.Coast);
      m_MFR.setNeutralMode(NeutralMode.Coast);
      m_MBL.setNeutralMode(NeutralMode.Coast);
      m_MBR.setNeutralMode(NeutralMode.Coast);
    }
  }

  /**
   * Sets the motor voltage outputs and feeds the drivetrain forward
   * Used in Ramsete Command constructor, param number 9
   *
   * @param leftVolts  voltage on the left side
   * @param rightVolts voltage on the right side
   */
  public void setOutput(double leftVolts, double rightVolts) {
    m_motorsLeft.setVoltage(leftVolts);
    m_motorsRight.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Gets absolute distance traveled by the left side of the robot in high gear
   *
   * @return The absolute distance of the left side in meters
   */
  private double getDistanceL() {
    return m_MFL.getSelectedSensorPosition() / Constants.Chassis.kEncoderResolution
            * (Constants.Chassis.kChassisGearRatio) * ((Constants.Chassis.kWheelDiameter) * Math.PI);
  }

  /**
   * Gets absolute distance traveled by the right side of the robot in high gear
   *
   * @return The absolute distance of the right side in meters
   */
  private double getDistanceR() {
    return m_MFR.getSelectedSensorPosition() / Constants.Chassis.kEncoderResolution
            * (Constants.Chassis.kChassisGearRatio) * ((Constants.Chassis.kWheelDiameter) * Math.PI);
  }


  /**
   * Returns the current speed of the front left motor in high gear
   *
   * @return Current speed of the front left motor (meters per second)
   */
  public double getSpeedL() {
    return (m_MFL.getSelectedSensorVelocity() / Constants.Chassis.kEncoderResolution
            * (Constants.Chassis.kChassisGearRatio) * (Math.PI * Constants.Chassis.kWheelDiameter)) * 10;
  }

  /**
   * Returns the current speed of the front right motor in high gear
   *
   * @return Current speed of the front right motor (meters per second)
   */
  public double getSpeedR() {
    return (m_MFR.getSelectedSensorVelocity() / Constants.Chassis.kEncoderResolution
            * (Constants.Chassis.kChassisGearRatio) * (Math.PI * Constants.Chassis.kWheelDiameter)) * 10;
  }

  /**
   * Returns the current speed of the robot by averaging the front left and right
   * motors
   *
   * @return Current speed of the robot
   */
  public double getSpeed() {
    return 0.5 * (getSpeedL() + getSpeedR());
  }

  private void resetEncoders() {
    m_MFL.setSelectedSensorPosition(0);
    m_MFR.setSelectedSensorPosition(0);
  }

  /**
   * Configure the maximum ramping rate of the drivetrain while in Open Loop
   * control mode
   * <p>
   * Value of 0 disables ramping
   *
   * @param maxRampRateSeconds Minimum desired time to go from neutral to full
   *                           throttle
   */
  public void configRampRate(double maxRampRateSeconds) {
    m_MFR.configOpenloopRamp(maxRampRateSeconds);
    m_MFL.configOpenloopRamp(maxRampRateSeconds);
    m_MBR.configOpenloopRamp(maxRampRateSeconds);
    m_MBL.configOpenloopRamp(maxRampRateSeconds);
  }

  /**
   * Output values to shuffleboard
   */
  public void outputToShuffleboard() {
    SmartDashboard.putNumber("Navx Heading", Navx.getHeading());
  }
  public double getNavxRotation(){ return Navx.getHeading(); }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Chassis");
    builder.addDoubleProperty("Navx rotation", this::getNavxRotation, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Feeds the bot forward
   * Used in ramsete command constructor param number 4
   * 
   * @return {@link SimpleMotorFeedforward} object that is used in Ramsete command
   */
  public SimpleMotorFeedforward getFeedforward() {
      return m_feedforward;
  }

  /**
   * Gets wheel speeds
   * Used in Ramsete Command Constructor, param num 6
   * 
   * @return wheel speeds as a {@link DifferentialDrive} object
   */
  public DifferentialDriveWheelSpeeds getSpeeds() {
      return new DifferentialDriveWheelSpeeds(getSpeedL(), getSpeedR());
  }

  /**
   * used in Ramsete Command constructor, param number 7
   * 
   * @return left pidcontroller
   */
  public PIDController getleftPIDController() {
      return m_leftPIDController;
  }

  public PIDController getRightPIDController() {
      return m_rightPIDConttroller;
  }
}

