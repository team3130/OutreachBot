// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Drive extends CommandBase {

  private final Chassis m_chassis;
  private final RobotContainer m_robotContainer;

  /**
   * Creates a new Command.
   * @param subsystem The subsystem used by this command.
   */
  public Drive(Chassis subsystem, RobotContainer container) {
    m_chassis = subsystem;
    m_robotContainer = container;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  /** ALL THE FOLLOWING ARE BUILT-IN METHODS BASED ON WHEN THE COMMAND IS RUN**/
  @Override
  public void initialize() {  //called once when the command is scheduled
    m_chassis.configRampRate(Constants.Chassis.kMaxRampRate);}

  @Override
  public void execute() {   //called continuously/repeatedly when the command is scheduled
    double moveSpeed = -RobotContainer.m_driverGamepad.getRawAxis(1); //joystick axis value (inverted)
    double turnSpeed = -RobotContainer.m_driverGamepad.getRawAxis(4) * .65; //joystick axis value (inverted and scaled down)
    m_chassis.driveArcade(moveSpeed,turnSpeed,true); //method to drive
  }

  @Override
  public void end(boolean interrupted) {   //called once the command ends or is interrupted
    m_chassis.configRampRate(0);} //disables ramping

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
