package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignToPose {
    public static Command create(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        return drivetrain.driveToPose(targetPose);
    }
}
