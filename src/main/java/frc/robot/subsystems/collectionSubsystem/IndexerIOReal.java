package frc.robot.subsystems.collectionSubsystem;

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
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IndexerConstants;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.KickerConstants;
import frc.robot.utils.RobotConstants;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX spindexerMotor = new TalonFX(
        IndexerConstants.MOTOR_ID,
        RobotConstants.CANIVORE
    );
    private final SparkFlex leaderMotor = new SparkFlex(
        KickerConstants.KICKER_LEADER_ID,
        MotorType.kBrushless
    );
    private final SparkFlex followerMotor = new SparkFlex(
        KickerConstants.KICKER_FOLLOWER_ID,
        MotorType.kBrushless
    );
    private final DutyCycleOut request = new DutyCycleOut(0).withEnableFOC(true);

    public IndexerIOReal() {
        var talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits
            .withStatorCurrentLimit(IndexerConstants.STATOR_LIMIT)
            .withStatorCurrentLimitEnable(true);
        talonConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        spindexerMotor.getConfigurator().apply(talonConfig);

        var leaderConfig = new SparkFlexConfig()
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(KickerConstants.CURRENT_LIMIT);

        var followerConfig = new SparkFlexConfig()
            .follow(leaderMotor, false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(KickerConstants.CURRENT_LIMIT);

        leaderMotor.configure(
            leaderConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        followerMotor.configure(
            followerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.spindexerAppliedVolts = spindexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.kickerAppliedVolts = leaderMotor.getBusVoltage() * leaderMotor.getAppliedOutput();
    }

    @Override
    public void set(double kickerOutput, double spindexerOutput) {
        leaderMotor.set(kickerOutput);
        spindexerMotor.setControl(request.withOutput(spindexerOutput));
    }
}
