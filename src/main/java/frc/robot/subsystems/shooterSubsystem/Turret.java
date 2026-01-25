package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
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

public class Turret extends SubsystemBase {

    TalonFX turretmotor = new TalonFX(1); //id subject to change
    SmartMotorControllerConfig motorconfig = new SmartMotorControllerConfig(
        this
    )
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            4,
            0,
            0,
            DegreesPerSecond.of(180),
            DegreesPerSecondPerSecond.of(90)
        )
        //need specific mechanical setup
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withMotorInverted(false)
        .withTelemetry("TurretMotor", TelemetryVerbosity.LOW)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));
    SmartMotorController motor = new TalonFXWrapper(
        turretmotor,
        DCMotor.getKrakenX60(1),
        motorconfig
    ); //change to Kraken X44

    PivotConfig pivotConfig = new PivotConfig(motor)
        .withStartingPosition(Degrees.of(0))
        .withHardLimit(Degrees.of(0), Degrees.of(355))
        .withTelemetry("PivotTelemetry", TelemetryVerbosity.LOW)
        .withMOI(Meters.of(0.25), Pounds.of(4));

    private Pivot turret = new Pivot(pivotConfig);
    private Angle targetAngle = Degrees.of(0);

    public Turret() {}

    public void setTargetAngle(Angle angle) {
        turret.setAngle(angle);
        targetAngle = angle;
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Boolean atPosition() {
        return targetAngle.isNear(turret.getAngle(), Degrees.of(1));
    }
}
