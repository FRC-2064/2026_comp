package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;
import frc.robot.utils.RobotConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {

    private final TalonFX flywheelMotor = new TalonFX(FlyWheelConstants.LEADER_ID);
    private final TalonFX followerMotor = new TalonFX(FlyWheelConstants.FOLLOWER_ID);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                    FlyWheelConstants.kP,
                    FlyWheelConstants.kI,
                    FlyWheelConstants.kD)
            .withGearing(FlyWheelConstants.GEARING)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(FlyWheelConstants.STATOR_LIMIT)
            .withClosedLoopRampRate(FlyWheelConstants.RAMP_RATE)
            .withOpenLoopRampRate(FlyWheelConstants.RAMP_RATE)
            .withFollowers(new Pair<>(followerMotor, true))
            .withMotorInverted(true)
            .withTelemetry("ShooterMotor", RobotConstants.GetTelemetry());

    private final SmartMotorController motor = new TalonFXWrapper(
            flywheelMotor,
            FlyWheelConstants.MOTORS,
            motorConfig);

    private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(motor)
            .withDiameter(FlyWheelConstants.WHEEL_DIAMETER)
            .withMass(FlyWheelConstants.WHEEL_MASS)
            .withLowerSoftLimit(FlyWheelConstants.MIN_VELOCITY)
            .withUpperSoftLimit(FlyWheelConstants.MAX_VELOCITY)
            .withTelemetry("Shooter", RobotConstants.GetTelemetry());

    private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

    private AngularVelocity targetSpeed = RPM.of(0);

public FlywheelSubsystem() {
        setDefaultCommand(flywheel.setSpeed(() -> this.targetSpeed));
    }

    public void setTargetSpeed(AngularVelocity speed) {
        this.targetSpeed = speed;
    }

    public boolean isUpToSpeed() {
        return flywheel.isNear(targetSpeed, FlyWheelConstants.TOLERANCE).getAsBoolean();
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
