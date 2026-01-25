package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {

    TalonFX hoodMotor = new TalonFX(1); //can change id
    SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(
        this
    )
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            4,
            0,
            0,
            DegreesPerSecond.of(10),
            DegreesPerSecondPerSecond.of(5)
        ) //PID and degreesPerSecond subject to change
        .withFeedforward(new ArmFeedforward(0, 0, 0))
        //gear ratios subject to change
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withMotorInverted(false)
        .withTelemetry("HoodMotor", TelemetryVerbosity.LOW)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));
    SmartMotorController motor = new TalonFXWrapper(
        hoodMotor,
        DCMotor.getKrakenX60(1),
        motorConfig
    ); //change to X44

    ArmConfig hoodConfig = new ArmConfig(motor)
        .withStartingPosition(Degrees.of(0)) //can be changed to reflect design
        .withHardLimit(Degrees.of(-30), Degrees.of(40))
        .withSoftLimits(Degrees.of(-30), Degrees.of(40))
        .withTelemetry("Hood", TelemetryVerbosity.HIGH);

    private Arm hood = new Arm(hoodConfig);
    private Angle targetAngle = Degrees.of(0);

    public Hood() {}

    public Angle getCurrentAngle() {
        return hood.getAngle();
    }

    public void setTargetAngle(Angle angle) {
        hood.setAngle(angle);
        targetAngle = angle;
    }

    public boolean atPosition() {
        return targetAngle.isNear(hood.getAngle(), Degrees.of(1));
    }
}
