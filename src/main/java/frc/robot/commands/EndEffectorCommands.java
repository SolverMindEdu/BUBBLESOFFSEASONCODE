package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorCommands {
    /** Run roller at given speed */
    public static InstantCommand run(EndEffectorSubsystem effector, double speed) {
        return new InstantCommand(() -> effector.runRoller(speed), effector);
    }

    /** Stop roller */
    public static InstantCommand stop(EndEffectorSubsystem effector) {
        return new InstantCommand(effector::stop, effector);
    }
}
