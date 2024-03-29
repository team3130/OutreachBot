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

  public static final boolean joystick = true;
  public static final boolean xbox = false;
  public static boolean functionalityMode = false;

  public static boolean limited = true;
  public static boolean all = false;
  
  public static class CAN {
    public static final int SHOOTERFLYWHEEL = 6;
    public static final int LEFTINDEXER = 7;
    public static final int RIGHTINDEXER = 8;
    public static final int frontLeftDrive = 2; // TO-DO
    public static final int frontRightDrive = 3; // TO-DO
    public static final int backLeftDrive = 4; // TO-DO
    public static final int backRightDrive = 5; // TO-DO

    public static final int Intake_Motor = 9; // TO-DO

    public static final int PCM = 0;
    public static final int PDP = 1;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class XBOXButtons {
    /**
 * Gamepad Button List
 */
  public static final int A = 1;
  public static final int B = 2;
  public static final int X = 3;
  public static final int Y = 4;
  public static final int LBUMPER = 5;
  public static final int RBUMPER = 6;
  public static final int WINDOW = 7;
  public static final int MENU = 8;
  public static final int LJOYSTICKPRESS = 9;
  public static final int RJOYSTICKPRESS = 10;

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

  public static class Chassis {
    //Constants for Chassis
    public static final int maxVoltage = 8; // TODO
    //max voltage for chassis motors
    public static final int encoderResolution = 0; // TODO
    public static final int gearRatio = 0; // TODO
    public static final int wheelDiameter = 0; // TODO

    public static final int kDriveDeadband = 0; // TODO

    public static final int kEncoderResolution = 0; // TODO
    public static final int kChassisGearRatio = 0; // TODO
    public static final int kWheelDiameter = 0; // TODO

    public static double kMaxRampRate = 0.7;

    // Navx

    public static final boolean kNavxReversed = false; // TODO

    public static class CAN {
      //CAN IDs for Chassis
      //All CAN IDs for Bot
      public static final int frontLeftDrive = 2; // TO-DO
      public static final int frontRightDrive = 3; // TO-DO
      public static final int backLeftDrive = 4; // TO-DO
      public static final int backRightDrive = 5; // TO-DO
    }
  }

  public static class Shooter {
    //Constants for Shooter
  }
  public static class Intake {
    //Constants for Intake
  }
}
