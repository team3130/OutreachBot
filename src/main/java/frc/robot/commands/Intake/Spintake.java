// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Spintake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final Intake m_intake;

  /**
   * Creates a new Command.
   * @param subsystem The subsystem used by this command.
   */
  public Spintake(Intake subsystem) {
    m_intake = subsystem;
    addRequirements(subsystem);
  }

  /** ALL THE FOLLOWING ARE BUILT-IN METHODS BASED ON WHEN THE COMMAND IS RUN**/
  @Override
  public void initialize() {  // called once when the command is scheduled
    m_intake.updateEnableVoltageCompensation(m_intake.getVoltageCompBoolean());
    m_intake.updateVoltageCompensationNum(m_intake.getVoltNum());
  }

  @Override  //called continuously/repeatedly when the command is scheduled
  public void execute() {
    m_intake.spinIntake();
  }

  @Override //called once the command ends or is interrupted
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //returns true when the command should end
    return false;
  }
}
