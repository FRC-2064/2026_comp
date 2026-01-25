package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeConstants;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase {
    private final TalonFX wristMotor = new TalonFX(IntakeConstants.WRIST_ID);
    private final Pivot wrist;
    
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);
    
    public enum DesiredState { INTAKE, STOWED, OUTTAKE }
    private DesiredState desiredState = DesiredState.STOWED;

    public Intake() {
        this.wrist = new Pivot(IntakeConstants.WRIST_CONFIG.withSmartMotorController(
            new TalonFXWrapper(wristMotor, IntakeConstants.WRIST_MOTOR_TYPE, IntakeConstants.WRIST_MOTOR_CONFIG.withSubsystem(this))
        ));

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_LIMIT.in(Amps);
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    public void setDesiredState(DesiredState newState) {
        if (this.desiredState == newState) {
            return;
        }

        this.desiredState = newState;

        switch (newState) {
            case INTAKE:
                wrist.setAngle(IntakeConstants.INTAKE_ANGLE);
                rollerMotor.set(IntakeConstants.INTAKE_SPEED);
                break;
            case OUTTAKE:
                wrist.setAngle(IntakeConstants.INTAKE_ANGLE);
                rollerMotor.set(IntakeConstants.OUTTAKE_SPEED);
                break;
            case STOWED:
                wrist.setAngle(IntakeConstants.STOW_ANGLE);
                rollerMotor.set(0);
                break;
        }
    }

    public DesiredState getDesiredState() {
        return desiredState;
    }

    @Override
    public void periodic() {
        wrist.updateTelemetry();
        SmartDashboard.putString("Intake/DesiredState", desiredState.toString());
    }

    @Override
    public void simulationPeriodic() {
        wrist.simIterate();
    }
}