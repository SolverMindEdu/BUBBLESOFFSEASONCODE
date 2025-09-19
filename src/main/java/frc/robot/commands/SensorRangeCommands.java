package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.EECoralRangeSubsystem;
import frc.robot.subsystems.AlgaeDetectRangeSubsystem;
import frc.robot.subsystems.IntakeRangeSubsystem;

public class SensorRangeCommands {
    /** Wait until coral sensor detects target in range */
    public static Command waitForCoral(EECoralRangeSubsystem sensor, double minMm, double maxMm) {
        return new WaitUntilCommand(() -> sensor.isWithinRange(minMm, maxMm));
    }

    /** Wait until algae sensor detects target in range */
    public static Command waitForAlgae(AlgaeDetectRangeSubsystem sensor, double minMm, double maxMm) {
        return new WaitUntilCommand(() -> sensor.isWithinRange(minMm, maxMm));
    }

    /** Wait until third sensor detects target in range */
    public static Command waitForIntake(IntakeRangeSubsystem sensor, double minMm, double maxMm) {
        return new WaitUntilCommand(() -> sensor.isWithinRange(minMm, maxMm));
    }
}
