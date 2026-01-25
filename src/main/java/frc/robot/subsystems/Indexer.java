package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private TalonFX indexerMotor = new TalonFX(1);
    private TalonFXConfiguration config = new TalonFXConfiguration();

    public Indexer() {
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerMotor.getConfigurator().apply(config);
    }

    public void feed() {
        indexerMotor.set(0.9);
    }

    public void outtake() {
        indexerMotor.set(-0.9);
    }

    public void stop() {
        indexerMotor.stopMotor();
    }
}
