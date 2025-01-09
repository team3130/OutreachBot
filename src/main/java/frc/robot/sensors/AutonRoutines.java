package frc.robot.sensors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

public class AutonRoutines {

    private Chassis chassis;

    private Intake intake;

    public AutonRoutines(Chassis chassis, Intake intake) {
        this.chassis = chassis;
        this.intake = intake;
    }

    public Command driveAndIntake() {
        return Commands.sequence();
                        //intake.takey());
    }
}