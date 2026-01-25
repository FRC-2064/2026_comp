package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IndexerConstants;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(IndexerConstants.MOTOR_ID);

    public Indexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerMotor.getConfigurator().apply(config);
    }

    public void feed() {
        indexerMotor.set(IndexerConstants.FEED_SPEED);
    }

    public void outtake() {
        indexerMotor.set(IndexerConstants.OUTTAKE_SPEED);
    }

    public void stop() {
        indexerMotor.stopMotor();
    }
}