package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    /** Horizontal offset (degrees) */
    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /** Vertical offset (degrees) */
    public double getTy() {
        return table.getEntry("ty").getDouble(0.0);
    }

    /** Target area (%) */
    public double getTa() {
        return table.getEntry("ta").getDouble(0.0);
    }

    /** Example yaw for 3D alignment */
    public double getYaw() {
        // If you're using botpose, yaw is index 5 (rotation around Z)
        double[] pose = table.getEntry("botpose").getDoubleArray(new double[6]);
        return pose.length >= 6 ? pose[5] : 0.0;
    }
}
