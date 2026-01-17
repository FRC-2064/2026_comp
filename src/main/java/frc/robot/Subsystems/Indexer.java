package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(1);
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    
    private Indexer() {
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    public void feed() {
        indexerMotor.set(0.9);
    }

    public void outtake() {
        indexerMotor.set(-0.9);
    }
}
