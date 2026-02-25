package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;
import frc.robot.utils.RobotConstants;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {

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
            .withGearing(HoodConstants.GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(HoodConstants.STATOR_LIMIT)
            .withTelemetry("HoodMotor", RobotConstants.GetTelemetry());

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
        .withTelemetry("Hood", RobotConstants.GetTelemetry());

    private final Pivot hood = new Pivot(hoodConfig);

    private final Debouncer statorDebounce = new Debouncer(0.1);

    private Angle targetAngle = HoodConstants.STARTING_POS;

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

    private boolean atHardStop() {
        return statorDebounce.calculate(motor.getStatorCurrent().gte(HoodConstants.STATOR_LIMIT));
    }

    public Command home() {
        return Commands.run(
            () -> hood.setVoltage(Volts.of(-5)), this)
        .until(this::atHardStop)
        .andThen(() -> motor.setEncoderPosition(HoodConstants.MIN_ANGLE));
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
