// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autos;
import frc.robot.commands.Intake.Resetake;
import frc.robot.commands.Intake.Spouttake;
import frc.robot.commands.LED.*;
import frc.robot.commands.chassis.FaceTarget;
import frc.robot.commands.chassis.Drive;
import frc.robot.commands.chassis.PullOut;
import frc.robot.subsystems.*;
import frc.robot.commands.Intake.Spintake;
import frc.robot.commands.Intake.Spouttake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.Unshoot;

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
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  protected SendableChooser<String> m_chooser_controller;
  protected  SendableChooser<String> m_chooser_functionality;




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

  public static Joystick m_Gamepad = new Joystick(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public void configureBindings() {
      /** Shooter **/
//  if (m_chassis.getJoystickName().equals("Logitech Extreme 3D")){
//    new JoystickButton(m_Gamepad, 1).whileTrue(new Shoot(m_shooter));
//  }
//  else if (m_chassis.getJoystickName().equals("Controller (Xbox One For Windows)")){
//    new JoystickButton(m_Gamepad,  Constants.XBOXButtons.Y).whileTrue(new Shoot(m_shooter));
//  }

      /** Intake **/
//  if (m_chassis.getJoystickName().equals("Logitech Extreme 3D")){
//    new JoystickButton(m_Gamepad, 2).whileTrue(new Spintake(m_intake));
//    new JoystickButton(m_Gamepad, 3).whileTrue(new Spouttake(m_intake));
//    new JoystickButton(m_Gamepad, 5).whileTrue(new Resetake(m_intake));
//  }
//
//  else if (m_chassis.getJoystickName().equals("Controller (Xbox One For Windows)")){
//      new JoystickButton(m_Gamepad, Constants.XBOXButtons.X).whileTrue(new Spintake(m_intake));
//      new JoystickButton(m_Gamepad, Constants.XBOXButtons.A).whileTrue(new Spouttake(m_intake));
//      new JoystickButton(m_Gamepad, Constants.XBOXButtons.LBUMPER).whileTrue(new Resetake(m_intake));
//      new JoystickButton(m_Gamepad, Constants.XBOXButtons.RBUMPER).whileTrue(new FaceTarget(m_chassis));
//    }
      /** LEDS **/

      if (m_chassis.getJoystickName().equals("Logitech Extreme 3D")) {
          new JoystickButton(m_Gamepad, 3).whileTrue(new Red(ledSubsystem));
          new JoystickButton(m_Gamepad, 5).whileTrue(new Green(ledSubsystem));
          new JoystickButton(m_Gamepad, 4).whileTrue(new Blue(ledSubsystem));
          new JoystickButton(m_Gamepad, 6).whileTrue(new MovingRainbow(ledSubsystem));
          new JoystickButton(m_Gamepad, 1).whileTrue(new Custom(ledSubsystem));
      }
      else if (m_chassis.getJoystickName().equals("Controller (Xbox One For Windows)")) {
          new JoystickButton(m_Gamepad, Constants.XBOXButtons.LST_POV_N).whileTrue(new Red(ledSubsystem));
          new JoystickButton(m_Gamepad, Constants.XBOXButtons.LST_POV_E).whileTrue(new Green(ledSubsystem));
          new JoystickButton(m_Gamepad, Constants.XBOXButtons.LST_POV_S).whileTrue(new Blue(ledSubsystem));
          new JoystickButton(m_Gamepad, Constants.XBOXButtons.LST_AXS_RTRIGGER).whileTrue(new MovingRainbow(ledSubsystem));
          new JoystickButton(m_Gamepad, Constants.XBOXButtons.LST_AXS_LTRIGGER).whileTrue(new Custom(ledSubsystem));
      }
  }

  public void vomitShuffleBoardData() {
      ShuffleboardTab chassis = Shuffleboard.getTab("Chassis");
      chassis.add(m_chassis);
      ShuffleboardTab intake = Shuffleboard.getTab("Intake");
      intake.add(m_intake);
      ShuffleboardTab shooter = Shuffleboard.getTab("Shooter");
      shooter.add(m_shooter);
      ShuffleboardTab LEDs = Shuffleboard.getTab("LEDSubsystem");
      shooter.add(ledSubsystem);

  }
  public RobotContainer(SendableChooser<String> functionalityChooser, SendableChooser<String> controllerChooser) {

    m_chooser_controller = controllerChooser;
    m_chooser_functionality = functionalityChooser;
    configureBindings();
    //m_chassis.setDefaultCommand(new Drive(m_chassis, this));
    ledSubsystem.setDefaultCommand(new JoystickLED(ledSubsystem));
    vomitShuffleBoardData();
  }
  public CommandBase pullOut(){
    return new PullOut(m_chassis);
  }

  public String returnController(){
    return m_chooser_controller.getSelected();
  }
  public String returnFunctionality(){
    return m_chooser_functionality.getSelected();
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
