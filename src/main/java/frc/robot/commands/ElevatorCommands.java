package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommands {
    public static Command reset(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.Reset), elevator),
            new WaitForElevatorCommand(elevator, ElevatorSubsystem.Reset),
            new InstantCommand(() -> {
                if (elevator.atBottom()) {
                    elevator.setZero(); // snap encoder to 0
                }
            }, elevator)
        );
    }

    public static Command intake(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.Intake), elevator),
            new WaitForElevatorCommand(elevator, ElevatorSubsystem.Intake)
        );
    }

    public static Command goToLevel1(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.LEVEL1), elevator),
            new WaitForElevatorCommand(elevator, ElevatorSubsystem.LEVEL1)
        );
    }

    public static Command goToLevel2(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.LEVEL2), elevator),
            new WaitForElevatorCommand(elevator, ElevatorSubsystem.LEVEL2)
        );
    }

    public static Command goToLevel3(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.LEVEL3), elevator),
            new WaitForElevatorCommand(elevator, ElevatorSubsystem.LEVEL3)
        );
    }

    public static Command goToLevel4(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.LEVEL4), elevator),
            new WaitForElevatorCommand(elevator, ElevatorSubsystem.LEVEL4)
        );
    }

    public static Command Algae1(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.Algae1), elevator),
            new WaitForElevatorCommand(elevator, ElevatorSubsystem.Algae1)
        );
    }

    public static Command Algae2(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.Algae2), elevator),
            new WaitForElevatorCommand(elevator, ElevatorSubsystem.Algae2)
        );
    }

    public static InstantCommand stop(ElevatorSubsystem elevator) {
        return new InstantCommand(elevator::stop, elevator);
    }
}
