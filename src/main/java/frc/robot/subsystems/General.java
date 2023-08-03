// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class General extends SubsystemBase {
  /** Subsystem for general use */
  public General() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public boolean getControllerType(){
    return Constants.controllerType;
  }
  public void setControllerType(boolean type){
    Constants.controllerType = type;
  }
  public boolean getFunctionalityMode(){
    return Constants.functionalityMode;
  }
  public void setFunctionalityMode(boolean type){
    Constants.functionalityMode = type;
  }
  @Override
  public void initSendable(SendableBuilder builder) {// outputs to shuffleboard in a way that can be update-able in real time (many of these can be removed after testing)
    builder.addBooleanProperty("Controller Type (T=joystick, F=xbox)", this::getControllerType, this::setControllerType);
    builder.addBooleanProperty("Functionality (T=limited, F=all", this::getControllerType, this::setFunctionalityMode);
  }
}
