// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class Red extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem ledSubsystem;
  private final Timer timer;
  private double frequency = 1/10; //frequency of moving rainbow is 10 times per second;

  /**
   * Creates a new ExampleCommand.
   *
   * @param // The subsystem used by this command.
   */
  public Red(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(ledSubsystem.getLimitSwitch()) {
      ledSubsystem.movingRainbow(20);
    }
    else {
      ledSubsystem.reset();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.reset();
    timer.start();
    if  (timer.hasElapsed(frequency))
    if(ledSubsystem.getLimitSwitch()) {
      ledSubsystem.movingRainbow(ledSubsystem.getNextFirstPixelColor(ledSubsystem.getFirstPixelColor()));
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
