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
    System.out.println("x: " + x + "  y: " + y + "\n");

    double radius = Math.sqrt((y*y) + (x*x));
    if (radius < 10) {
      radius = 0;
    }
    radius = Math.pow(radius, 2) % 255;

    double theta = ((Math.atan2(y, x) + (2 * Math.PI))) / (2 * Math.PI);
    int color = (int) (theta * 180);

    subsystem.setCustom(color, 255, (int) radius);
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