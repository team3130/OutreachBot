// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.LED;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;


/** An example command that uses an example subsystem. */
public class Green extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem ledSubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param // The subsystem used by this command.
   */
  public Green(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!ledSubsystem.getLimitSwitch()) {
      ledSubsystem.green();
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ledSubsystem.getLimitSwitch()) {
      ledSubsystem.green();
    }
    else {
      ledSubsystem.reset();
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.reset();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}