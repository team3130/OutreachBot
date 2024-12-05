// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EthanWheel extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public EthanWheel() {}
private final WPI_TalonSRX my_motor;

  private final int CANDID = 1;

  my_motor = new WPI_TalonSRX(CANID);

  public void run_motor() {
    my_motor.
  }
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
