package frc.robot.subsystems.collectionSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeConstants;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase {
    private final TalonFX extendMotor = new TalonFX(IntakeConstants.EXTEND_ID);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                    IntakeConstants.kP,
                    IntakeConstants.kI,
                    IntakeConstants.kD,
                    IntakeConstants.MAX_VEL,
                    IntakeConstants.MAX_ACCEL)
            .withFeedforward(IntakeConstants.FEEDFORWARD)
            .withSimClosedLoopController(
                    IntakeConstants.kP,
                    IntakeConstants.kI,
                    IntakeConstants.kD,
                    IntakeConstants.MAX_VEL,
                    IntakeConstants.MAX_ACCEL)
            .withSimFeedforward(IntakeConstants.FEEDFORWARD)
            .withGearing(IntakeConstants.RACK_GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(IntakeConstants.STATOR_LIMIT)
            .withClosedLoopRampRate(IntakeConstants.RAMP_RATE)
            .withTelemetry("IntakeMotor", TelemetryVerbosity.LOW);

    private final SmartMotorController motor = new TalonFXWrapper(
            extendMotor, IntakeConstants.MOTOR_TYPE, motorConfig);
  
    private final ElevatorConfig extenderConfig = new ElevatorConfig(motor)
            .withStartingHeight(IntakeConstants.STOW_HEIGHT)
            .withHardLimits(IntakeConstants.STOW_HEIGHT, IntakeConstants.INTAKE_HEIGHT)
            .withMass(IntakeConstants.MOI_MASS)
            .withTelemetry("Intake", TelemetryVerbosity.LOW);

    private final Elevator rack = new Elevator(extenderConfig);

    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);

    public enum IntakeState {
        INTAKE, 
        STOWED,
        DEPLOYED,
        OUTTAKE
    }

    private IntakeState desiredState = IntakeState.STOWED;

    public Intake() {
    }

    public void setDesiredState(IntakeState newState) {
        if (this.desiredState == newState) {
            return;
        }

        this.desiredState = newState;

        switch (newState) {
            case INTAKE:
                rack.setHeight(IntakeConstants.INTAKE_HEIGHT);
                rollerMotor.set(IntakeConstants.INTAKE_SPEED);
                break;
            case OUTTAKE:
                rack.setHeight(IntakeConstants.INTAKE_HEIGHT);
                rollerMotor.set(IntakeConstants.OUTTAKE_SPEED);
                break;
            case STOWED:
                rack.setHeight(IntakeConstants.STOW_HEIGHT);
                rollerMotor.set(0);
                break;
            case DEPLOYED:
                wrist.setAngle(IntakeConstants.INTAKE_ANGLE);
                rollerMotor.set(0);
                break;
        }
    }

    public IntakeState getDesiredState() {
        return desiredState;
    }

    @Override
    public void periodic() {
        rack.updateTelemetry();
        SmartDashboard.putString("Intake/DesiredState", desiredState.toString());
    }

    @Override
    public void simulationPeriodic() {
        rack.simIterate();
    }
}