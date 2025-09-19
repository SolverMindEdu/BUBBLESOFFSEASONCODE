package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeRollersSubsystems extends SubsystemBase {
    private final TalonFX roller = new TalonFX(Configs.CAN.intakerollers);

    public IntakeRollersSubsystems() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // Invert if motor is spinning the wrong direction
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        roller.getConfigurator().apply(configs);
    }

    /** Run roller at given percent output (-1.0 to 1.0) */
    public void runRoller(double speed) {
        roller.set(speed);
    }

    /** Stop roller */
    public void stop() {
        roller.stopMotor();
    }
}
