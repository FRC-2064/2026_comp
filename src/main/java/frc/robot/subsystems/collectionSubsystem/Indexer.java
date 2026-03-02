package frc.robot.subsystems.collectionSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IndexerConstants;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.KickerConstants;
import frc.robot.utils.RobotConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Indexer extends SubsystemBase {
    private final TalonFX spindexerMotor = new TalonFX(IndexerConstants.MOTOR_ID, RobotConstants.CANIVORE);
    private final SparkFlex leaderMotor = new SparkFlex(KickerConstants.KICKER_LEADER_ID, MotorType.kBrushless);
    private final SparkFlex followerMotor = new SparkFlex(KickerConstants.KICKER_FOLLOWER_ID, MotorType.kBrushless);

    private final SmartMotorControllerConfig spindexerConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withStatorCurrentLimit(IndexerConstants.STATOR_LIMIT)
    .withIdleMode(MotorMode.COAST)
    .withMotorInverted(true)
    .withGearing(4)
    .withTelemetry("SpindexerMotor", RobotConstants.GetTelemetry());

    private final SmartMotorControllerConfig kickerConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withMotorInverted(true)
    .withIdleMode(MotorMode.COAST)
    .withFollowers(Pair.of(followerMotor, false))
    .withGearing(1)
    .withTelemetry("KickerMotor", RobotConstants.GetTelemetry());

    private final SmartMotorController spindexer = new TalonFXWrapper(spindexerMotor, DCMotor.getKrakenX44Foc(1), spindexerConfig);
    private final SmartMotorController kicker = new SparkWrapper(leaderMotor, DCMotor.getNeoVortex(2), kickerConfig);

    public void feed() {
        kicker.setDutyCycle(KickerConstants.FEED);
        spindexer.setDutyCycle(IndexerConstants.FEED);
    }

    public void outtake() {
        kicker.setDutyCycle(KickerConstants.OUTTAKE);
        spindexer.setDutyCycle(IndexerConstants.OUTTAKE);
    }

    public void stop() {
        kicker.setDutyCycle(0);
        spindexer.setDutyCycle(0);
    }
}
