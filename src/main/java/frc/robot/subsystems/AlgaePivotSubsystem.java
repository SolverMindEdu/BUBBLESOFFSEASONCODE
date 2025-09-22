package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class AlgaePivotSubsystem extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(Configs.CAN.algaepivet, "EndEffector");

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    // Example pivot angles in rotations (tune these!)
    public static final double RESET = 0;
    public static final double intakeMove = 2;
    public static final double ElevatorMove = 0.9;
    public static final double Algae = 8;
    public static final double Process = 5;

    public AlgaePivotSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // Invert if needed
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // PID
        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = 8;   // tune
        slot0.kI = 0;
        slot0.kD = 0;

        // Motion Magic limits
        configs.MotionMagic.MotionMagicCruiseVelocity = 10;  // slow for safety
        configs.MotionMagic.MotionMagicAcceleration   = 20;

        pivotMotor.getConfigurator().apply(configs);

        // Reset position at startup
        pivotMotor.setPosition(0);
    }

    /** Move pivot to position in rotations */
    public void setPosition(double rotations) {
        pivotMotor.setControl(motionMagic.withPosition(rotations));
    }

    /** Stop pivot */
    public void stop() {
        pivotMotor.stopMotor();
    }

    /** Read pivot position */
    public double getPosition() {
        return pivotMotor.getRotorPosition().getValue().in(Rotations);
    }
}
