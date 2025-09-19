package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeRollersSubsystems;

public class IntakeRollersCommands {
    /** Run roller at given speed */
    public static InstantCommand run(IntakeRollersSubsystems intakerollers, double speed) {
        return new InstantCommand(() -> intakerollers.runRoller(speed), intakerollers);
    }

    /** Stop roller */
    public static InstantCommand stop(IntakeRollersSubsystems intakerollers) {
        return new InstantCommand(intakerollers::stop, intakerollers);
    }
}
