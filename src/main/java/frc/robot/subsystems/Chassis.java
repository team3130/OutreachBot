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
  
  private final WPI_TalonFX m_frontLeftDrive; //motor-front-left
  private final WPI_TalonFX m_backLeftDrive;
  private final WPI_TalonFX m_backRightDrive;
  private final WPI_VictorSPX m_frontRightDrive;

  private final DifferentialDrive m_drive;

  private final MotorControllerGroup m_motorsRight;
  private final MotorControllerGroup m_motorsLeft;

  private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDConttroller;

  private final Navx m_navx = Navx.GetInstance();

  
  public Chassis() {
    m_frontLeftDrive = new WPI_TalonFX(Constants.CAN.frontLeftDrive);
    m_frontRightDrive = new WPI_VictorSPX(Constants.CAN.frontRightDrive);
    m_backLeftDrive = new WPI_TalonFX(Constants.CAN.backLeftDrive);
    m_backRightDrive = new WPI_TalonFX(Constants.CAN.backRightDrive);
    
    m_frontLeftDrive.configFactoryDefault();
    m_frontRightDrive.configFactoryDefault();
    m_backLeftDrive.configFactoryDefault();
    m_backRightDrive.configFactoryDefault();
    
    m_frontLeftDrive.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_frontRightDrive.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_backLeftDrive.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_backRightDrive.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
      
    m_frontLeftDrive.enableVoltageCompensation(true);
    m_frontRightDrive.enableVoltageCompensation(true);
    m_backLeftDrive.enableVoltageCompensation(true);
    m_backRightDrive.enableVoltageCompensation(true);

    m_frontRightDrive.setInverted(true);
    m_backRightDrive.setInverted(true);
    m_backLeftDrive.setInverted(false);
    m_frontLeftDrive.setInverted(false);

    m_motorsRight = new MotorControllerGroup(m_frontRightDrive, m_backRightDrive);
    m_motorsLeft = new MotorControllerGroup(m_frontLeftDrive, m_backLeftDrive);

    m_drive = new DifferentialDrive(m_motorsLeft, m_motorsRight);
    m_drive.setDeadband(Constants.Chassis.kDriveDeadband);
    m_drive.setSafetyEnabled(false);

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
      m_frontLeftDrive.setNeutralMode(NeutralMode.Brake);
      m_frontRightDrive.setNeutralMode(NeutralMode.Brake);
      m_backLeftDrive.setNeutralMode(NeutralMode.Brake);
      m_backRightDrive.setNeutralMode(NeutralMode.Brake);
    } else {
      m_frontLeftDrive.setNeutralMode(NeutralMode.Coast);
      m_frontRightDrive.setNeutralMode(NeutralMode.Coast);
      m_backLeftDrive.setNeutralMode(NeutralMode.Coast);
      m_backRightDrive.setNeutralMode(NeutralMode.Coast);
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
    return m_frontLeftDrive.getSelectedSensorPosition() / Constants.Chassis.kEncoderResolution
            * (Constants.Chassis.kChassisGearRatio) * ((Constants.Chassis.kWheelDiameter) * Math.PI);
  }

  /**
   * Gets absolute distance traveled by the right side of the robot in high gear
   *
   * @return The absolute distance of the right side in meters
   */
  private double getDistanceR() {
    return m_frontRightDrive.getSelectedSensorPosition() / Constants.Chassis.kEncoderResolution
            * (Constants.Chassis.kChassisGearRatio) * ((Constants.Chassis.kWheelDiameter) * Math.PI);
  }


  /**
   * Returns the current speed of the front left motor in high gear
   *
   * @return Current speed of the front left motor (meters per second)
   */
  public double getSpeedL() {
    return (m_frontLeftDrive.getSelectedSensorVelocity() / Constants.Chassis.kEncoderResolution
            * (Constants.Chassis.kChassisGearRatio) * (Math.PI * Constants.Chassis.kWheelDiameter)) * 10;
  }

  /**
   * Returns the current speed of the front right motor in high gear
   *
   * @return Current speed of the front right motor (meters per second)
   */
  public double getSpeedR() {
    return (m_frontRightDrive.getSelectedSensorVelocity() / Constants.Chassis.kEncoderResolution
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
    m_frontLeftDrive.setSelectedSensorPosition(0);
    m_frontRightDrive.setSelectedSensorPosition(0);
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
    m_frontRightDrive.configOpenloopRamp(maxRampRateSeconds);
    m_frontLeftDrive.configOpenloopRamp(maxRampRateSeconds);
    m_backRightDrive.configOpenloopRamp(maxRampRateSeconds);
    m_backLeftDrive.configOpenloopRamp(maxRampRateSeconds);

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

