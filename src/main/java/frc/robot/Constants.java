// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Drivetrain{
    public static double maxVoltage = 9.0;
    public static double inputDeadband = 0.02;
    public static double rampRate = 0.7; // seconds to full output
    public static double turnSpeed = 0.8;
    public static double slowingScalar = 0.55;

  }

  public static class CAN {
    public static int leftFrontDrivetrain = 0;
    public static int rightFrontDrivetrain = 1;
    public static int leftBackDrivetrain = 2;
    public static int rightBackDrivetrain = 3;
    public static int drivetrainLeftEncoder = 7;
    public static int drivetrainRightEncoder = 8;
    public static int flywheelShooter = 4;
    public static int indexerShooter = 5;
    public static int intakeBeaterBar = 6;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class Buttons {
    /**
 * Gamepad Button List
 */
  public static final int LST_BTN_A = 1;
  public static final int LST_BTN_B = 2;
  public static final int LST_BTN_X = 3;
  public static final int LST_BTN_Y = 4;
  public static final int LST_BTN_LBUMPER = 5;
  public static final int LST_BTN_RBUMPER = 6;
  public static final int LST_BTN_WINDOW = 7;
  public static final int LST_BTN_MENU = 8;
  public static final int LST_BTN_LJOYSTICKPRESS = 9;
  public static final int LST_BTN_RJOYSTICKPRESS = 10;

/**
 * Gamepad POV List
 */
  public static final int LST_POV_UNPRESSED = -1;
  public static final int LST_POV_N = 0;
  public static final int LST_POV_NE = 45;
  public static final int LST_POV_E = 90;
  public static final int LST_POV_SE = 135;
  public static final int LST_POV_S = 180;
  public static final int LST_POV_SW = 225;
  public static final int LST_POV_W = 270;
  public static final int LST_POV_NW = 315;

/**
 * Gamepad Axis List
 */
  public static final int LST_AXS_LJOYSTICKX = 0;
  public static final int LST_AXS_LJOYSTICKY = 1;
  public static final int LST_AXS_LTRIGGER = 2;
  public static final int LST_AXS_RTRIGGER = 3;
  public static final int LST_AXS_RJOYSTICKX = 4;
  public static final int LST_AXS_RJOYSTICKY = 5;
}
}
