package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class AlgaeDetectRangeSubsystem extends SubsystemBase {
    private final CANrange sensor = new CANrange(Configs.CAN.AlgaeDetect, "EndEffector");

    public double getDistanceMillimeters() {
        return sensor.getDistance().getValueAsDouble() * 1000.0;
    }

    public boolean isWithinRange(double minMm, double maxMm) {
        double dist = getDistanceMillimeters();
        return dist >= minMm && dist <= maxMm;
    }
}
