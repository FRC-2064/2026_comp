package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public enum HomingState {
        NOT_HOMED,
        HOMING,
        HOMED,
        HOMING_FAILED,
    }

    private final TalonFX hoodMotor = new TalonFX(HoodConstants.MOTOR_ID);

    private final SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                HoodConstants.kP,
                HoodConstants.kI,
                HoodConstants.kD,
                HoodConstants.MAX_VELOCITY,
                HoodConstants.MAX_ACCELERATION
            )
            .withFeedforward(HoodConstants.FEEDFORWARD)
            .withSimClosedLoopController(
                HoodConstants.kP,
                HoodConstants.kI,
                HoodConstants.kD,
                HoodConstants.MAX_VELOCITY,
                HoodConstants.MAX_ACCELERATION
            )
            .withSimFeedforward(HoodConstants.FEEDFORWARD)
            .withGearing(HoodConstants.GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(HoodConstants.STATOR_LIMIT)
            .withClosedLoopRampRate(HoodConstants.RAMP_RATE)
            .withTelemetry("HoodMotor", TelemetryVerbosity.LOW);

    private final SmartMotorController motor = new TalonFXWrapper(
        hoodMotor,
        HoodConstants.MOTOR_TYPE,
        motorConfig
    );

    private final PivotConfig hoodConfig = new PivotConfig(motor)
        .withStartingPosition(HoodConstants.STARTING_POS)
        .withHardLimit(HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE)
        .withSoftLimits(HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE)
        .withMOI(HoodConstants.MOI_LENGTH, HoodConstants.MOI_MASS)
        .withTelemetry("Hood", TelemetryVerbosity.LOW);

    private final Pivot hood = new Pivot(hoodConfig);

    private Angle targetAngle = HoodConstants.STARTING_POS;
    private HomingState homingState = HomingState.NOT_HOMED;

    private Time homingTimer = Seconds.zero();
    private Time stallTimer = Seconds.zero();

    public Hood() {
        setDefaultCommand(hood.setAngle(() -> this.targetAngle));
    }

    public void setTargetAngle(Angle angle) {
        this.targetAngle = angle;
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return hood.getAngle();
    }

    public boolean atPosition() {
        return hood.isNear(targetAngle, HoodConstants.TOLERANCE).getAsBoolean();
    }

    public HomingState getHomingState() {
        return homingState;
    }

    public boolean isHomed() {
        return homingState == HomingState.HOMED;
    }

    public Command homingSequence() {
        return runOnce(() -> {
            homingState = HomingState.HOMING;
            homingTimer = Seconds.zero();
            stallTimer = Seconds.zero();
        })
            .andThen(
                run(() -> {
                    homingTimer.plus(Milliseconds.of(20));
                    motor.setVoltage(HoodConstants.HOMING_VOLTAGE);
                    var currentDraw = motor.getStatorCurrent();

                    if (currentDraw.gt(HoodConstants.HOMING_STALL_CURRENT)) {
                        stallTimer.plus(Milliseconds.of(20));
                    } else {
                        stallTimer = Seconds.zero();
                    }
                })
            )
            .until(() -> {
                if (stallTimer.gte(HoodConstants.HOMING_STALL_TIME)) {
                    return true;
                }
                if (homingTimer.gte(HoodConstants.HOMING_TIMEOUT)) {
                    return true;
                }

                return false;
            })
            .andThen(() -> {
                motor.setVoltage(Volts.zero());

                if (stallTimer.gte(HoodConstants.HOMING_STALL_TIME)) {
                    motor.setEncoderPosition(HoodConstants.HOMING_OFFSET);
                    homingState = HomingState.HOMED;
                    targetAngle = HoodConstants.HOMING_OFFSET;
                } else {
                    homingState = HomingState.HOMING_FAILED;
                }
            })
            .withName("HoodHoming");
    }

    public Command manualHoming() {
        return runOnce(() -> {
            homingState = HomingState.HOMED;
            motor.setEncoderPosition(HoodConstants.MIN_ANGLE);
            targetAngle = HoodConstants.STARTING_POS;
        }).withName("ManualHome");
    }

    public void resetHomingState() {
        homingState = HomingState.NOT_HOMED;
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();

        SmartDashboard.putString("Hood/HomingState", homingState.name());
        SmartDashboard.putNumber("Hood/HomingTimer", homingTimer.in(Seconds));
        SmartDashboard.putNumber("Hood/StallTimer", stallTimer.in(Seconds));
        SmartDashboard.putBoolean("Hood/IsHomed", isHomed());
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();

        if (homingState == HomingState.NOT_HOMED && DriverStation.isEnabled()) {
            homingState = HomingState.HOMED;
        }
    }
}
