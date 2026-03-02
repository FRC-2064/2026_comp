package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeConstants;
import frc.robot.utils.RobotConstants;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeExtension extends SubsystemBase {
    private final TalonFX extendMotor = new TalonFX(IntakeConstants.EXTEND_ID, RobotConstants.CANIVORE);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                    IntakeConstants.kP,
                    IntakeConstants.kI,
                    IntakeConstants.kD,
                    IntakeConstants.MAX_VEL,
                    IntakeConstants.MAX_ACCEL)
            .withGearing(IntakeConstants.RACK_GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(IntakeConstants.STATOR_LIMIT)
            .withTelemetry("ExtensionMotor", RobotConstants.GetTelemetry())
            .withMechanismCircumference(Inches.of(4.4));

    private final SmartMotorController motor = new TalonFXWrapper(
            extendMotor, IntakeConstants.MOTOR_TYPE, motorConfig);

    private final ElevatorConfig extenderConfig = new ElevatorConfig(motor)
            .withStartingHeight(IntakeConstants.STOW_HEIGHT)
            .withHardLimits(IntakeConstants.STOW_HEIGHT, IntakeConstants.INTAKE_HEIGHT)
            .withMass(IntakeConstants.MOI_MASS)
            .withTelemetry("Extension", RobotConstants.GetTelemetry());

    private final Elevator rack = new Elevator(extenderConfig);

    private boolean extended = false;

    public IntakeExtension() {}

    public void extend() {
        //rack.setHeight(IntakeConstants.INTAKE_HEIGHT);
        motor.setDutyCycle(0.75);
        extended = true;
    }

    public void stow() {
        //rack.setHeight(IntakeConstants.STOW_HEIGHT);
        motor.setDutyCycle(-0.75);
        extended = false;
    }

    public void setMotorZero(){
        motor.setEncoderPosition(Meters.zero());
    }

    @Override
    public void periodic() {
        rack.updateTelemetry();
        SmartDashboard.putNumber("extension/height", rack.getHeight().in(Inches));
        SmartDashboard.putBoolean("extension/extende", extended);
    }

    @Override
    public void simulationPeriodic() {
        rack.simIterate();
    }
}
