// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.general.SwitchControllerMode;
import frc.robot.commands.general.SwitchFunctionalityMode;
import frc.robot.commands.chassis.Drive;
import frc.robot.commands.shooter.RunFlywheel;
import frc.robot.commands.shooter.RunIndexers;
import frc.robot.subsystems.*;
import frc.robot.commands.Intake.Spintake;
import frc.robot.commands.Intake.SpoutTake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.Unshoot;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Chassis m_chassis = new Chassis();
  private final Intake m_intake = new Intake();
  private final General m_general = new General();
  private final XboxController m_Gamepad = new XboxController(0);



  /* Replace with CommandPS4Controller or CommandJoystick if needed
  private final Chassis m_chassis = new Chassis();
  private final Chassis m_chassis = new Chassis();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static Joystick m_joystick = new Joystick(0);
  private final XboxController m_Gamepad = new XboxController(0);


  /* Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      
   */

  public static Joystick m_driverGamepad = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_chassis.setDefaultCommand(new Drive(m_chassis, this));

    vomitShuffleBoardData();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.co
   * mmand.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /* Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand()); */


  /** Shooter **/
    new JoystickButton(m_Gamepad, 1).whileTrue(new Shoot(m_shooter));
    new JoystickButton(m_Gamepad, 4).whileTrue(new Unshoot(m_shooter));
    new POVButton(m_Gamepad, Constants.Buttons.LST_POV_N).whileTrue(new RunFlywheel(m_shooter));
    new POVButton(m_Gamepad, Constants.Buttons.LST_POV_S).whileTrue(new RunIndexers(m_shooter));

  /** Intake **/
    new JoystickButton(m_Gamepad, 2).whileTrue(new Spintake(m_intake));
    new JoystickButton(m_Gamepad, 3).whileTrue(new SpoutTake(m_intake));

    /** General **/
    new JoystickButton(m_Gamepad, 12).whileTrue(new SwitchControllerMode(m_general));
    new JoystickButton(m_Gamepad, 8).whileTrue(new SwitchFunctionalityMode(m_general));


  }

  public void vomitShuffleBoardData() {
      ShuffleboardTab chassis = Shuffleboard.getTab("Chassis");
      chassis.add(m_chassis);
      ShuffleboardTab intake = Shuffleboard.getTab("Intake");
      intake.add(m_intake);
      ShuffleboardTab shooter = Shuffleboard.getTab("Shooter");
      shooter.add(m_shooter);
  }

  /**
   * Use this to pass the autonomous
   * command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
