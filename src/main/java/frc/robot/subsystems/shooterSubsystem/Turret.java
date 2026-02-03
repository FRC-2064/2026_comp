package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {

    private SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.MAX_VEL,
                TurretConstants.MAX_ACCEL
            )
            .withFeedforward(TurretConstants.FEEDFORWARD)
            .withSimClosedLoopController(
                15,
                0,
                2,
                TurretConstants.MAX_VEL,
                TurretConstants.MAX_ACCEL
            )
            .withSimFeedforward(TurretConstants.FEEDFORWARD)
            .withGearing(TurretConstants.GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(TurretConstants.STATOR_LIMIT)
            .withClosedLoopRampRate(TurretConstants.RAMP_RATE)
            .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH);

    private final TalonFX turretMotor = new TalonFX(TurretConstants.MOTOR_ID);
    private final SmartMotorController motor = new TalonFXWrapper(
        turretMotor,
        TurretConstants.MOTOR_TYPE,
        motorConfig
    );

    public PivotConfig turretConfig = new PivotConfig(motor)
        .withStartingPosition(TurretConstants.STARTING_POS)
        .withHardLimit(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE)
        .withSoftLimits(Degrees.of(5), Degrees.of(350))
        .withTelemetry("Turret", TelemetryVerbosity.HIGH)
        .withMOI(TurretConstants.LENGTH, TurretConstants.WEIGHT)
        .withMechanismPositionConfig(
            new MechanismPositionConfig()
            .withMaxRobotHeight(Inches.of(22))
            .withMaxRobotLength(Inches.of(27))
            .withMovementPlane(Plane.XY)
        );

    private final Pivot turret = new Pivot(turretConfig);

    private Angle targetAngle = Degrees.of(0);

    public Turret() {
        setDefaultCommand(turret.setAngle(() -> this.targetAngle));
    }

    public void setTargetAngle(Angle angle) {
        this.targetAngle = angle;
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return turret.getAngle();
    }

    public boolean atPosition() {
        return turret.isNear(targetAngle, TurretConstants.TOLERANCE).getAsBoolean();
    }

    @Override
    public void periodic() {
        turret.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        turret.simIterate();
    }
}