package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class RunEndEffectorCommand extends Command {
    private final EndEffectorSubsystem endEffector;
    private final double speed;

    public RunEndEffectorCommand(EndEffectorSubsystem endEffector, double speed) {
        this.endEffector = endEffector;
        this.speed = speed;
        addRequirements(endEffector); // ensures subsystem isn’t used by 2 commands at once
    }

    @Override
    public void initialize() {
        endEffector.runRoller(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop only if you want it to stop after command finishes
        // If you want it to keep running, just leave this empty
        endEffector.stop();
    }
}
