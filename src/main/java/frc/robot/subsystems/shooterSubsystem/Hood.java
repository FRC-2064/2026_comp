package frc.robot.subsystems.shooterSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {

    private final TalonFX hoodMotor = new TalonFX(HoodConstants.MOTOR_ID);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                    HoodConstants.kP,
                    HoodConstants.kI,
                    HoodConstants.kD,
                    HoodConstants.MAX_VELOCITY,
                    HoodConstants.MAX_ACCELERATION)
            .withFeedforward(HoodConstants.FEEDFORWARD)
            .withSimClosedLoopController(
                    HoodConstants.kP,
                    HoodConstants.kI,
                    HoodConstants.kD,
                    HoodConstants.MAX_VELOCITY,
                    HoodConstants.MAX_ACCELERATION)
            .withSimFeedforward(HoodConstants.FEEDFORWARD)
            .withGearing(HoodConstants.GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(HoodConstants.STATOR_LIMIT)
            .withClosedLoopRampRate(HoodConstants.RAMP_RATE)
            .withTelemetry("HoodMotor", TelemetryVerbosity.LOW);

    private final SmartMotorController motor = new TalonFXWrapper(
            hoodMotor,
            HoodConstants.MOTOR_TYPE,
            motorConfig);

    private final PivotConfig hoodConfig = new PivotConfig(motor)
            .withStartingPosition(HoodConstants.STARTING_POS)
            .withHardLimit(HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE)
            .withSoftLimits(HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE)
            .withMOI(HoodConstants.MOI_LENGTH, HoodConstants.MOI_MASS)
            .withTelemetry("Hood", TelemetryVerbosity.LOW);

    private final Pivot hood = new Pivot(hoodConfig);

    private Angle targetAngle = HoodConstants.STARTING_POS;

    public Hood() {
    }

    public void setTargetAngle(Angle angle) {
        this.targetAngle = angle;
        hood.setAngle(angle);
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return hood.getAngle();
    }

    public boolean atPosition() {
        return targetAngle.isNear(hood.getAngle(), HoodConstants.TOLERANCE);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}