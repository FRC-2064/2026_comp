package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
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

public class Turret extends SubsystemBase {

    private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                    TurretConstants.kP,
                    TurretConstants.kI,
                    TurretConstants.kD,
                    TurretConstants.MAX_VEL,
                    TurretConstants.MAX_ACCEL)
            .withFeedforward(TurretConstants.FEEDFORWARD)
            .withSimClosedLoopController(
                    TurretConstants.kP,
                    TurretConstants.kI,
                    TurretConstants.kD,
                    TurretConstants.MAX_VEL,
                    TurretConstants.MAX_ACCEL)
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
            motorConfig);

    public PivotConfig turretConfig = new PivotConfig(motor)
            .withStartingPosition(TurretConstants.STARTING_POS)
            .withHardLimit(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE)
            .withSoftLimits(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE)
            .withTelemetry("Turret", TelemetryVerbosity.HIGH)
            .withMOI(TurretConstants.LENGTH, TurretConstants.WEIGHT);

    private final Pivot turret = new Pivot(turretConfig);

    private Angle targetAngle = Degrees.of(0);

    public Turret() {
    }

    public void setTargetAngle(Angle angle) {
        targetAngle = angle;
        turret.setAngle(angle);
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return turret.getAngle();
    }

    public boolean atPosition() {
        return targetAngle.isNear(turret.getAngle(), TurretConstants.TOLERANCE);
    }

    @Override
    public void periodic() {
        turret.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        turretMotor.getSimState().setSupplyVoltage(12.0);
        turret.simIterate();
    }
}