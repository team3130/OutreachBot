// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class ControlledShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter The subsystem used by this command.
   */
  public ControlledShoot(Shooter shooter) {
    m_shooter = shooter;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    m_shooter.runFlywheelControlled(m_shooter.getCurrentFlywheelNative(), m_shooter.getRPMSetpointInNative());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.runFlywheelControlled(m_shooter.getCurrentFlywheelNative(), m_shooter.getRPMSetpointInNative());
    if (timer.hasElapsed(0.2)){
      m_shooter.spinIndexer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFlywheel();
    m_shooter.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
