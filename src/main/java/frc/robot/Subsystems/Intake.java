package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.EnumMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase {
    TalonFX wristMotor = new TalonFX(1);//can change device ID
    SmartMotorControllerConfig wristMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withMotorInverted(false)
        .withTelemetry("TurretMotor", TelemetryVerbosity.LOW)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));
    SmartMotorController motor = new TalonFXWrapper(wristMotor, DCMotor.getKrakenX60(1), wristMotorConfig);

    PivotConfig wristConfig = new PivotConfig(motor)
    .withStartingPosition(Degrees.of(0))
    .withHardLimit(Degrees.of(0), Degrees.of(355))
    .withTelemetry("PivotTelemetry", TelemetryVerbosity.LOW)
    .withMOI(Meters.of(0.25), Pounds.of(4)); //can change weight


    private Pivot wrist = new Pivot(wristConfig);
    private TalonFX intakeMotor = new TalonFX(1);
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private DesiredState desiredState = DesiredState.STOWED;
    private CurrentState currentState = CurrentState.STOWED;
    private final EnumMap<DesiredState, Runnable> stateActions;

    public Intake() {
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotor.getConfigurator().apply(config);
        stateActions = new EnumMap<>(DesiredState.class);
        stateActions.put(DesiredState.INTAKE, this::intake);
        stateActions.put(DesiredState.STOWED, this::stow);
        stateActions.put(DesiredState.OUTAKE, this::outake);
    }

    public enum DesiredState{
        INTAKE,
        STOWED,
        OUTAKE
    }
    public enum CurrentState{
        INTAKING,
        STOWING,
        STOWED,
        OUTAKING
    }

    private void intake(){
        wrist.setAngle(Degrees.of(0));
        intakeMotor.set(1);
    }
    private void stow(){
        wrist.setAngle(Degrees.of(0));
        intakeMotor.set(0);
    }
    private void outake(){
        wrist.setAngle(Degrees.of(0));
        intakeMotor.set(-1);
    }
    public void setDesiredState(DesiredState state){
        desiredState = state;
    }
    public CurrentState getCurrentState(){
        return currentState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/IntakeState", getCurrentState().toString());
    }

    public void setState(DesiredState newState){
        if(desiredState == newState){
            return;
        }

        desiredState = newState;
        Runnable action = stateActions.get(newState);
        if (action != null){
            action.run();
        }
    }
}