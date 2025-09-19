package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class WaitForElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double target;

    public WaitForElevatorCommand(ElevatorSubsystem elevator, double target) {
        this.elevator = elevator;
        this.target = target;
    }

    @Override
    public boolean isFinished() {
        return elevator.atTarget(target);
    }
}
