package frc.robot.commands.LED;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickLED extends CommandBase {
  private final LEDSubsystem subsystem;

  private final XboxController xboxController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickLED(LEDSubsystem subsystem) {
    this.subsystem = subsystem;
    xboxController = new XboxController(0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = xboxController.getRawAxis(Constants.XBOXButtons.LST_AXS_LJOYSTICKY);
    double x = xboxController.getRawAxis(Constants.XBOXButtons.LST_AXS_LJOYSTICKX);
    y *= 100;
    x *= 100;

    double radius = Math.sqrt((y*y) + (x*x));
    double theta = 0;

    try {
      theta = Math.tan(y/x);
    } catch(ArithmeticException e) {
      if (y > 0) {
        theta = 90;
      } else if (y < 0) {
        theta = 270;
      } else {
        theta = 0;
      }
    }

    subsystem.setCustom((int)(theta / 2), 255, (int)(radius + 155));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}