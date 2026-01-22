import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class flywheel extends SubsystemBase
{
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3,4)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST).withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25));
    
    private TalonFX flywheelMotor = new TalonFX(0);
    private SmartMotorController motorController = new TalonFXWrapper(flywheelMotor, DCMotor.getKrakenX60Foc(1), smcConfig);

    private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(motorController)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(1))
    .withUpperSoftLimit(RPM.of(1000))
    .withTelemetry("Shooter", TelemetryVerbosity.HIGH);


    private FlyWheel flywheel = new FlyWheel(flywheelConfig);

    public flywheel()
    {

    }

    public AngularVelocity getVelocity()
    {
        return flywheel.getSpeed();
    }

    public Command setVelocity(AngularVelocity speed)
    {
        return flywheel.setSpeed(speed);
    }

    public Command set(double dutyCycle)
    {
        return flywheel.set(dutyCycle);
    }

    @Override
    public void periodic()
    {
        flywheel.updateTelemetry();
    }

    @Override
    public void simulatePeriodic()
    {
        flywheel.simIterate();
    }
}
