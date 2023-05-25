// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis m_chassis;
  private final RobotContainer m_robotcontainer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param chassis The subsystem used by this command.
   */
  public DefaultDrive(Chassis chassis, RobotContainer robotContainer) {
    m_chassis = chassis;
    m_robotcontainer = robotContainer;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.configRampRate();
    m_chassis.enableFollow(m_chassis.getFollower()); //to be removed once tested
    m_chassis.configureBreakMode(m_chassis.getBrake()); //to be removed once tested
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double moveSpeed = -RobotContainer.m_driverGamepad.getRawAxis(1); //joystick's y-axis is inverted
    moveSpeed *= Constants.Drivetrain.slowingScalar;

    double turnSpeed = RobotContainer.m_driverGamepad.getRawAxis(4) * (Constants.Drivetrain.turnSpeed);

    m_chassis.driveArcade(moveSpeed, turnSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.configEndRampRate(); //this is sketchy
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
