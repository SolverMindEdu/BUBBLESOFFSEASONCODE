package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

/**
 * Subsystem for the end effector roller.
 * Controls intake/outtake of algae/coral/etc. using a TalonFX.
 */
public class EndEffectorSubsystem extends SubsystemBase {
    private final TalonFX roller = new TalonFX(Configs.CAN.endeffectorroller, "EndEffector");

    public EndEffectorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // Invert motor if needed (CounterClockwise_Positive / Clockwise_Positive)
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        roller.getConfigurator().apply(configs);
    }

    /** Run roller at given percent output (-1.0 to 1.0). */
    public void runRoller(double speed) {
        roller.set(speed);
    }

    /** Stop roller safely. */
    public void stop() {
        roller.stopMotor();
    }

    /** Get the current applied output percentage. */
    public double getOutputPercent() {
        return roller.get();
    }

    /** Get motor current draw in amps. */
    public double getCurrent() {
        return roller.getSupplyCurrent().getValueAsDouble();
    }

    /** Get motor velocity (rotations per second). */
    public double getVelocity() {
        return roller.getRotorVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // Push telemetry to dashboard for debugging
        SmartDashboard.putNumber("EndEffector/Output %", getOutputPercent());
        SmartDashboard.putNumber("EndEffector/Current (A)", getCurrent());
        SmartDashboard.putNumber("EndEffector/Velocity (RPS)", getVelocity());
    }
}
