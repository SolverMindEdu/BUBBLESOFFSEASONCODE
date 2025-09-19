package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(Configs.CAN.ElevatorRight); // main motor
    private final TalonFX follower = new TalonFX(Configs.CAN.ElevatorLeft); // follows leader

    // Motion Magic request object
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    // Positions in rotations
    public static final double Reset  = 0.2;
    public static final double Intake = 1.1;
    public static final double LEVEL1 = 12.0;
    public static final double LEVEL2 = 20.0;
    public static final double LEVEL3 = 31.0;
    public static final double LEVEL4 = 50.0;
    public static final double Algae1 = 12;
    public static final double Algae2 = 32.0;

    // Tolerance for checking "at target"
    public static final double POSITION_TOLERANCE = 0.5;

    public ElevatorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // Motor inversion (change if needed)
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Motion Magic PID slot
        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = 5;   // tune!
        slot0.kI = 0.1;
        slot0.kD = 0;
        slot0.kV = 0;
        slot0.kS = 0;

        // Motion Magic settings (slowed for testing)
        configs.MotionMagic.MotionMagicCruiseVelocity = 25; // rotations/sec
        configs.MotionMagic.MotionMagicAcceleration   = 50; // rotations/sec^2

        leader.getConfigurator().apply(configs);

        // Reset encoder position to 0 whenever code boots
        leader.setPosition(0);

        // Follower follows leader (invert true if mounted opposite)
        follower.setControl(new com.ctre.phoenix6.controls.Follower(
            Configs.CAN.ElevatorRight, true
        ));
    }

    /** Move elevator to given target rotations */
    public void setPosition(double rotations) {
        leader.setControl(motionMagic.withPosition(rotations));
    }

    /** Stop elevator immediately */
    public void stop() {
        leader.stopMotor();
    }

    /** Force reset encoder position to zero */
    public void setZero() {
        leader.setPosition(0);
    }

    /** Read current position in rotations */
    public double getPosition() {
        return leader.getRotorPosition().getValue().in(Rotations);
    }

    /** Returns true if elevator is within tolerance of target */
    public boolean atTarget(double target) {
        return Math.abs(getPosition() - target) <= POSITION_TOLERANCE;
    }

    /** Returns true if at physical bottom (safe to zero) */
    public boolean atBottom() {
        return getPosition() <= Reset + POSITION_TOLERANCE;
    }
}
