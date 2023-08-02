// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class RunFlywheel extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter The subsystem used by this command.
   */
  public RunFlywheel(Shooter shooter) {
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  @Override
  public void initialize() {// called once when the command is scheduled
  }

  @Override
  public void execute() {  //called continuously/repeatedly when the command is scheduled
    m_shooter.runFlywheel();
  }

  @Override
  public void end(boolean interrupted) { //called once the command ends or is interrupted
    m_shooter.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
