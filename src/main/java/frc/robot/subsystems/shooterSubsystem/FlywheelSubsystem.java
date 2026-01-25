package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                50,
                0,
                0,
                DegreesPerSecond.of(90),
                DegreesPerSecondPerSecond.of(45)
            )
            .withSimClosedLoopController(
                50,
                0,
                0,
                DegreesPerSecond.of(90),
                DegreesPerSecondPerSecond.of(45)
            )
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
            .withGearing(
                new MechanismGearing(GearBox.fromReductionStages(3, 4))
            )
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    private TalonFX flywheelMotor = new TalonFX(0);
    private SmartMotorController motorController = new TalonFXWrapper(
        flywheelMotor,
        DCMotor.getKrakenX60Foc(1),
        smcConfig
    );

    private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(
        motorController
    )
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(1))
        .withUpperSoftLimit(RPM.of(1000))
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    private FlyWheel flywheel = new FlyWheel(flywheelConfig);
    private AngularVelocity targetSpeed = RPM.of(0);

    public FlywheelSubsystem() {}

    public boolean isUpToSpeed() {
        return flywheel.getSpeed().isNear(targetSpeed, RPM.of(10));
    }

    public void setVelocity(AngularVelocity speed) {
        targetSpeed = speed;
        flywheel.setSpeed(speed);
    }

    public AngularVelocity getTargetSpeed() {
        return targetSpeed;
    }

    public AngularVelocity getVelocity() {
        return flywheel.getSpeed();
    }

    @Override
    public void periodic() {
        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
