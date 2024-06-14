// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.sensors.Navx;

import java.util.function.Consumer;

public class Chassis extends SubsystemBase {
  public String joystickName = DriverStation.getJoystickName(0);
  private final WPI_TalonFX m_frontLeftDrive; //front left drivetrain motor
  private final WPI_TalonFX m_backLeftDrive;
  private final WPI_TalonFX m_backRightDrive;
  private final WPI_VictorSPX m_frontRightDrive; //this motor uses a different motor controller aka a victor

  private final DifferentialDrive m_drive; // a built-in class for a method of driving

  private final MotorControllerGroup m_motorsRight; //houses all the motors on the right side of the drivetrain
  private final MotorControllerGroup m_motorsLeft; //both right and left are used as a parameter for DifferentialDrive methods

  /*private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;*/
  private double angle = 0;
  private double chassisSpinP = 0.0099;
  private double chassisSpinI = 0.000001;
  private double chassisSpinD = 0.0015;
  private final Consumer<Double[]> circleFixer;
  private final PIDController m_spinnyPID;



  private final Navx m_navx = Navx.GetInstance();

  
  public Chassis() {
    //instantiating all motors
    m_frontLeftDrive = new WPI_TalonFX(Constants.CAN.frontLeftDrive);
    m_backLeftDrive = new WPI_TalonFX(Constants.CAN.backLeftDrive);
    m_backRightDrive = new WPI_TalonFX(Constants.CAN.backRightDrive);
    m_frontRightDrive = new WPI_VictorSPX(Constants.CAN.frontRightDrive);

    //good practice to put all motors to default
    m_frontLeftDrive.configFactoryDefault();
    m_frontRightDrive.configFactoryDefault();
    m_backLeftDrive.configFactoryDefault();
    m_backRightDrive.configFactoryDefault();

    //good practice to set voltage comp saturation to help with battery loss
    m_frontLeftDrive.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_frontRightDrive.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_backLeftDrive.configVoltageCompSaturation(Constants.Chassis.maxVoltage);
    m_backRightDrive.configVoltageCompSaturation(Constants.Chassis.maxVoltage);

    //turning on voltage comp
    m_frontLeftDrive.enableVoltageCompensation(true);
    m_frontRightDrive.enableVoltageCompensation(true);
    m_backLeftDrive.enableVoltageCompensation(true);
    m_backRightDrive.enableVoltageCompensation(true);

    //inverting one side of the drivetrain since the motors are facing different directions
    m_frontRightDrive.setInverted(true);
    m_backRightDrive.setInverted(true);
    m_backLeftDrive.setInverted(false);
    m_frontLeftDrive.setInverted(false);

    //setting up the motor controller groups
    m_motorsRight = new MotorControllerGroup(m_frontRightDrive, m_backRightDrive);
    m_motorsLeft = new MotorControllerGroup(m_frontLeftDrive, m_backLeftDrive);

    //setting up the DifferentialDrive
    m_drive = new DifferentialDrive(m_motorsLeft, m_motorsRight);

    //setting up controller deadbands (so tiny tiny controller values don't move the robot)
    m_drive.setDeadband(Constants.Chassis.kDriveDeadband);

    //no one really knows
    m_drive.setSafetyEnabled(false);

    /*m_feedforward = new SimpleMotorFeedforward(Constants.Chassis.ChassiskS, Constants.Chassis.ChassiskV, Constants.Chassis.ChassiskA);
    m_leftPIDController = new PIDController(Constants.Chassis.LChassiskP, Constants.Chassis.LChassiskI, Constants.Chassis.LChassiskD);
    m_rightPIDConttroller = new PIDController(Constants.Chassis.RChassiskP, Constants.Chassis.RChassiskI, Constants.Chassis.RChassiskD);
    */
    m_spinnyPID = new PIDController(chassisSpinP, chassisSpinI, chassisSpinD);

    //setting motors to brake mode (resists motion)
    configureBrakeMode(true);

    circleFixer = (Double[] angle) -> {
      angle[0] = ((angle[0] % 360) + 360) % 360;
      angle[0] += ((angle[0] > 180) ? -360 : 0);
    };
  }

  /** GENERAL METHODS**/
  // used to drive the robot, parameters are joystick, joystick, if you want inputs squared (creates a smoother driving experience)
  public void driveArcade(double moveThrottle, double turnThrottle, boolean squaredInputs) {
    m_drive.arcadeDrive(moveThrottle, turnThrottle, squaredInputs);
  }

  public void resetPIDLoop() {
    m_spinnyPID.reset();
    tuneTolerance();
    m_spinnyPID.enableContinuousInput(-180, 180); //this makes degrees go [-180,180] rather than [0,360] or [0,infinity]
  }
  public void tuneTolerance() {
    m_spinnyPID.setTolerance(3.5, 0.02);
    m_spinnyPID.setIntegratorRange(-0.1, 0.1);
  }
  public void spinOutput() {
    driveArcade(0, -m_spinnyPID.calculate(getSpinnyAngle()), false);
  }
  public double getSpinnyAngle() {
    if (Navx.getNavxPresent()) {
      // normalize to positive 360
      double angle = ((Navx.getAngle() % 360) + 360) % 360;
      return (angle + ((angle > 180) ? -360 : 0));
    } else {
      return 0;
    }
  }
  public void driveForward(){
    m_drive.arcadeDrive(0.75, 0);
  }

  public void stopDriving(){
    m_drive.arcadeDrive(0,0);
  }
  public void updatePIDValues() { //make sure the PID vals in use are up to date from shuffleboard
    m_spinnyPID.setPID(getSpinnyP(), getSpinnyI(), getSpinnyD());
  }

  public double getMoveSpeedValue(double slider){
    return -slider;
  }
  public double getTurnSpeedValue(double slider){
    return -slider * 0.65;
  }

  //used to configure brake mode (resists motion) or coast mode (doesn't) on all drive motors
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

  //configures the maximum ramping rate of the drivetrain(value of 0 disables ramping)
  //parameter of maxRampRateSeconds is the desired time to go from neutral to full throttle
  public void configRampRate(double maxRampRateSeconds) {
    m_frontRightDrive.configOpenloopRamp(maxRampRateSeconds);
    m_frontLeftDrive.configOpenloopRamp(maxRampRateSeconds);
    m_backRightDrive.configOpenloopRamp(maxRampRateSeconds);
    m_backLeftDrive.configOpenloopRamp(maxRampRateSeconds);

  }

  public double moveSpeed(){
      return  -RobotContainer.m_Gamepad.getRawAxis(1); //joystick up axis value (inverted)
  }
  public double turnSpeed(){
      return  -RobotContainer.m_Gamepad.getRawAxis(2);
  }

  public double movingScalar(){
    double x = -RobotContainer.m_Gamepad.getRawAxis(3);
    double y = ((x+1)/2) * 1.5; // taking slider return from [-1,1] to [0,1] and then scaling it up to enable real ue of more of its range
    if (y>1){
      return 1;} //eliminating values>1
    if(y<0.6){
      return 0.6;}
    else return y;
    }

  public double turningScalar(){
    return movingScalar() * 0.9;
  }

  /** GETTERS AND SETTERS**/
  public double getNavxRotation(){ return Navx.getHeading(); } //return navx heading aka where the bot is facing [-180,180]
  public double getGoalAngle(){
    return angle;
  }
  public void setGoalAngle(double goal){
    angle = goal;
  }
  public double getSpinnyP(){
    return chassisSpinP;
  }
  public void setSpinnyP(double goal){
    chassisSpinP = goal;
  }
  public double getSpinnyI(){
    return chassisSpinI;
  }
  public void setSpinnyI(double goal){
    chassisSpinI = goal;
  }
  public double getSpinnyD(){
    return chassisSpinD;
  }
  public void setSpinnyD(double goal){
    chassisSpinD = goal;
  }
  public String getJoystickName(){
    return DriverStation.getJoystickName(0);
  }
  @Override
  public void initSendable(SendableBuilder builder) {// outputs to shuffleboard in a way that can be update-able in real time (many of these can be removed after testing)
    builder.addDoubleProperty("Navx rotation", this::getNavxRotation, null);
    //builder.addVariableTypeProperty("name to display", this:getter, if you want it to be editable-> this::setter else -> null);
    builder.addDoubleProperty("Goal Rotation", this::getGoalAngle, this::setGoalAngle);
    builder.addDoubleProperty("Spinny P", this::getSpinnyP, this::setSpinnyP);
    builder.addDoubleProperty("Spinny I", this::getSpinnyI, this::setSpinnyI);
    builder.addDoubleProperty("Spinny D", this::getSpinnyD, this::setSpinnyD);

    builder.addStringProperty("Controller Type", this::getJoystickName, null);
  }

  public void setSpinnySetPoint(double setpoint) {
    Double[] angle = new Double[] {setpoint}; //caleb kugel's circle stuff
    circleFixer.accept(angle); //""
    m_spinnyPID.setSetpoint(angle[0]); //???
  }


  @Override
  public void periodic() {
    // This built-in method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This built-in method will be called once per scheduler run during simulation
  }

}

