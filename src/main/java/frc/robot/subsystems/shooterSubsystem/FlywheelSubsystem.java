package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {

    private final TalonFX flywheelMotor = new TalonFX(FlyWheelConstants.LEADER_ID);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                    FlyWheelConstants.kP,
                    FlyWheelConstants.kI,
                    FlyWheelConstants.kD,
                    FlyWheelConstants.MAX_VEL_PROFILED,
                    FlyWheelConstants.MAX_ACCEL_PROFILED)
            .withFeedforward(FlyWheelConstants.FEEDFORWARD)
            .withSimClosedLoopController(
                    FlyWheelConstants.kP,
                    FlyWheelConstants.kI,
                    FlyWheelConstants.kD,
                    FlyWheelConstants.MAX_VEL_PROFILED,
                    FlyWheelConstants.MAX_ACCEL_PROFILED)
            .withSimFeedforward(FlyWheelConstants.FEEDFORWARD)
            .withGearing(FlyWheelConstants.GEARING)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(FlyWheelConstants.STATOR_LIMIT)
            .withClosedLoopRampRate(FlyWheelConstants.RAMP_RATE)
            .withOpenLoopRampRate(FlyWheelConstants.RAMP_RATE)
            .withFollowers(new Pair<>(FlyWheelConstants.FOLLOWER_ID, true))
            .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW);

    private final SmartMotorController motor = new TalonFXWrapper(
            flywheelMotor,
            FlyWheelConstants.MOTORS,
            motorConfig);

    private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(motor)
            .withDiameter(FlyWheelConstants.WHEEL_DIAMETER)
            .withMass(FlyWheelConstants.WHEEL_MASS)
            .withLowerSoftLimit(FlyWheelConstants.MIN_VELOCITY)
            .withUpperSoftLimit(FlyWheelConstants.MAX_VELOCITY)
            .withTelemetry("Shooter", TelemetryVerbosity.LOW);

    private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

    private AngularVelocity targetSpeed = RPM.of(0);

    public FlywheelSubsystem() {
    }

    public boolean isUpToSpeed() {
        return flywheel.getSpeed().isNear(targetSpeed, FlyWheelConstants.TOLERANCE);
    }

    public void setVelocity(AngularVelocity speed) {
        targetSpeed = speed;
        flywheel.setSpeed(speed);
    }

    public AngularVelocity getTargetSpeed() {
        return targetSpeed;
    }

    public AngularVelocity getVelocity() {
        return flywheel.getSpeed();
    }

    @Override
    public void periodic() {
        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}