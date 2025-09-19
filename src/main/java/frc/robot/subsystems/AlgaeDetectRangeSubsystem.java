package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class AlgaeDetectRangeSubsystem extends SubsystemBase {
    private final CANrange sensor = new CANrange(Configs.CAN.AlgaeDetect, "EndEffector");

    /**
     * Get the distance in millimeters from the CANrange sensor.
     */
    public double getDistanceMillimeters() {
        return sensor.getDistance().getValueAsDouble() * 1000.0;
    }

    /**
     * Check if the detected object is within a specific range.
     * 
     * @param minMm minimum distance in millimeters
     * @param maxMm maximum distance in millimeters
     * @return true if the distance is within the range, false otherwise
     */
    public boolean isWithinRange(double minMm, double maxMm) {
        double dist = getDistanceMillimeters();
        return dist >= minMm && dist <= maxMm;
    }

    /**
     * Simple "detected" check for algae.
     * Default threshold: less than 20 mm.
     */
    public boolean isDetected() {
        return getDistanceMillimeters() <= 20.0; // ✅ adjust threshold as needed
    }
}
