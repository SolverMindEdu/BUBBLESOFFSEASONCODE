package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class EECoralRangeSubsystem extends SubsystemBase {
    private final CANrange sensor = new CANrange(Configs.CAN.EECoral, "EndEffector");

    /** Get distance in millimeters */
    public double getDistanceMillimeters() {
        return sensor.getDistance().getValueAsDouble() * 1000.0;
    }

    /** Check if target is within a distance band (in mm) */
    public boolean isWithinRange(double minMm, double maxMm) {
        double dist = getDistanceMillimeters();
        return dist >= minMm && dist <= maxMm;
    }
}
