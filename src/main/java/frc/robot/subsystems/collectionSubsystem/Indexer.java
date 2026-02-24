package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IndexerConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.KickerConstants;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(IndexerConstants.MOTOR_ID);
    private final SparkFlex leader = new SparkFlex(KickerConstants.KICKER_LEADER_ID, MotorType.kBrushless);
    private final SparkFlex follower = new SparkFlex(KickerConstants.KICKER_FOLLOWER_ID, MotorType.kBrushless);

    private final SparkFlexConfig followerConfig = new SparkFlexConfig();
    private final SparkFlexConfig leaderConfig = new SparkFlexConfig();

    public Indexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerMotor.getConfigurator().apply(config);

        leaderConfig
                .smartCurrentLimit(40)
                .inverted(true)
                .idleMode(IdleMode.kCoast);

        followerConfig
                .apply(leaderConfig)
                .follow(leader, false);

        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void feed() {
        indexerMotor.set(IndexerConstants.FEED_SPEED);
        leader.set(KickerConstants.FEED_SPEED);
    }

    public void outtake() {
        indexerMotor.set(IndexerConstants.OUTTAKE_SPEED);
        leader.set(KickerConstants.OUTTAKE_SPEED);
    }

    public void stop() {
        indexerMotor.stopMotor();
        leader.stopMotor();
    }
}
