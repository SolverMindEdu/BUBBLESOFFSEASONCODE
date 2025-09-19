package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaePivotSubsystem;

public class AlgaePivotCommands {
    public static InstantCommand stow(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.RESET), algae);
    }

    public static InstantCommand elevatorMove(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.ElevatorMove), algae);
    }

    public static InstantCommand intakeMove(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.intakeMove), algae);
    }

    public static InstantCommand algae(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.Algae), algae);
    }

    public static InstantCommand stop(AlgaePivotSubsystem algae) {
        return new InstantCommand(algae::stop, algae);
    }
}
