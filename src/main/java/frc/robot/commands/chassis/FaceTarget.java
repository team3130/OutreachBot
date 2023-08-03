package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class FaceTarget extends CommandBase {
  // defining an instance to be used throughout the command and to be instantiated in the constructor of type parameter
  private final Chassis m_chassis;


  private Pose2d pose;

  public FaceTarget(Chassis chassis) {
    //mapping to object passed through parameter
    m_chassis = chassis;
    m_requirements.add(chassis);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    m_chassis.updatePIDValues();
    m_chassis.setSpinnySetPoint(m_chassis.getGoalAngle());
    m_chassis.resetPIDLoop();
  }

  @Override
  public void execute() {
    if (Constants.functionalityMode == Constants.all) {
      m_chassis.spinOutput();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_chassis.configRampRate(0);
  }
}