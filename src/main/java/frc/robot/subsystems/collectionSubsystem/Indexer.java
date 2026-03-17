package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.KickerConstants;
import frc.robot.utils.RobotConstants;

public class Indexer extends SubsystemBase {
    private final TalonFX spindexerMotor = new TalonFX(IndexerConstants.MOTOR_ID, RobotConstants.CANIVORE);
    private final SparkFlex leaderMotor = new SparkFlex(KickerConstants.LEADER_ID, MotorType.kBrushless);
    private final SparkFlex followerMotor = new SparkFlex(KickerConstants.FOLLOWER_ID, MotorType.kBrushless);

    private final DutyCycleOut sr = new DutyCycleOut(0).withEnableFOC(true);

    public Indexer() {
        var sc = new TalonFXConfiguration();
        sc.CurrentLimits.withStatorCurrentLimit(IndexerConstants.STATOR_LIMIT)
        .withStatorCurrentLimitEnable(true);
        sc.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

        spindexerMotor.getConfigurator().apply(sc);

        var lc = new SparkFlexConfig()
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) KickerConstants.STATOR_LIMIT.in(Amps));

        var fc = new SparkFlexConfig()
        .follow(leaderMotor, false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) KickerConstants.STATOR_LIMIT.in(Amps));

        leaderMotor.configure(lc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(fc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void feed() {
        leaderMotor.set(KickerConstants.FEED_SPEED);
        spindexerMotor.setControl(sr.withOutput(IndexerConstants.FEED_SPEED));
    }

    public void clear() {
        leaderMotor.set(KickerConstants.OUTTAKE_SPEED);
        spindexerMotor.setControl(sr.withOutput(0));
    }

    public void outtake() {
        leaderMotor.set(KickerConstants.OUTTAKE_SPEED);
        spindexerMotor.setControl(sr.withOutput(IndexerConstants.OUTTAKE_SPEED));
    }

    public void stop() {
        leaderMotor.set(0);
        spindexerMotor.setControl(sr.withOutput(0));
    }
}
