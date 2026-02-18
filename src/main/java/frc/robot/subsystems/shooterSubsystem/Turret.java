package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase {

    private CANcoder throughBoreSmall = new CANcoder(1);
    private CANcoder throughBoreLarge = new CANcoder(2);

    private EasyCRTConfig easyCRTConfig = new EasyCRTConfig(
        throughBoreSmall.getAbsolutePosition().asSupplier(),
        throughBoreLarge.getAbsolutePosition().asSupplier()
    )
        .withCommonDriveGear(1.0, 80, 13, 14)
        .withMechanismRange(
            TurretConstants.MIN_ANGLE,
            TurretConstants.MAX_ANGLE
        )
        .withAbsoluteEncoderOffsets(Rotations.of(0), Rotations.of(0))
        .withMatchTolerance(Rotations.of(0.06))
        .withAbsoluteEncoderInversions(false, false);

    private EasyCRT solver = new EasyCRT(easyCRTConfig);

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
        .withMOI(TurretConstants.LENGTH, TurretConstants.WEIGHT);

    private final Pivot turret = new Pivot(turretConfig);

    private Angle targetAngle = Degrees.of(0);

    public Turret() {
        solver
            .getAngleOptional()
            .ifPresent(mechAngle -> {
                motor.setEncoderPosition(mechAngle);
            });
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
        return turret
            .isNear(targetAngle, TurretConstants.TOLERANCE)
            .getAsBoolean();
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
