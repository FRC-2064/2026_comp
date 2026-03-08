package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import frc.robot.utils.RobotConstants;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase {

    private final TalonFX turretMotor = new TalonFX(TurretConstants.MOTOR_ID);
    private final CANcoder throughBoreSmall = new CANcoder(
        TurretConstants.ENCODER_13_ID
    );
    private final CANcoder throughBoreLarge = new CANcoder(
        TurretConstants.ENCODER_14_ID
    );

    private final EasyCRTConfig easyCRTConfig = new EasyCRTConfig(
        throughBoreSmall.getAbsolutePosition().asSupplier(),
        throughBoreLarge.getAbsolutePosition().asSupplier()
    )
        .withCommonDriveGear(1.0, 80, 13, 14)
        .withMechanismRange(
            TurretConstants.MIN_ANGLE,
            TurretConstants.MAX_ANGLE
        )
        .withAbsoluteEncoderOffsets(
            TurretConstants.ENCODER_13_OFFSET,
            TurretConstants.ENCODER_14_OFFSET
        )
        .withMatchTolerance(Rotations.of(0.06))
        .withAbsoluteEncoderInversions(false, false);

    private final EasyCRT solver = new EasyCRT(easyCRTConfig);

    private final SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.MAX_VEL,
                TurretConstants.MAX_ACCEL
            )
            .withGearing(TurretConstants.GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(TurretConstants.STATOR_LIMIT)
            .withTelemetry("TurretMotor", RobotConstants.GetTelemetry());

    private final SmartMotorController motor = new TalonFXWrapper(
        turretMotor,
        TurretConstants.MOTOR_TYPE,
        motorConfig
    );

    private final PivotConfig turretConfig = new PivotConfig(motor)
        .withStartingPosition(TurretConstants.STARTING_POS)
        .withHardLimit(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE)
        .withSoftLimits(Degrees.of(-90), Degrees.of(90))
        .withTelemetry("Turret", RobotConstants.GetTelemetry())
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

    public void zero() {
        turretMotor.setPosition(Degrees.zero());
    }

    public void setEncoderZero(){
        turretMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        turret.updateTelemetry();
        SmartDashboard.putNumber("turret/angle", turret.getAngle().in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        turret.simIterate();
    }
}
