// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UmarWheel extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final WPI_TalonSRX exampleMotor;
    private final int CANID = 1;
    public UmarWheel() {
        exampleMotor = new WPI_TalonSRX(CANID);
        exampleMotor.configFactoryDefault();
    }

    public void runMotor() {
        exampleMotor.set(ControlMode.PercentOutput, 0.5);
    }

    public void stopMotor() {
        exampleMotor.set(ControlMode.PercentOutput, 0);
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
