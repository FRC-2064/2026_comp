package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ShooterConstants {

    public static class FlyWheelConstants {
        public static final int LEADER_ID = 25;
        public static final int FOLLOWER_ID = 26;

        public static final DCMotor MOTORS = DCMotor.getKrakenX60Foc(2);
        public static final Distance WHEEL_DIAMETER = Inches.of(4);
        public static final Mass WHEEL_MASS = Pounds.of(4.15);
        public static final MechanismGearing GEARING = new MechanismGearing(1);

        public static final Current STATOR_LIMIT = Amps.of(40);
        public static final AngularVelocity MIN_VELOCITY = RPM.of(0);
        public static final AngularVelocity MAX_VELOCITY = RPM.of(1000);
        public static final AngularVelocity TOLERANCE = RPM.of(50);
        public static final Time RAMP_RATE = Seconds.of(0.25);

        public static final double kP = 50.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final AngularVelocity MAX_VEL_PROFILED = DegreesPerSecond.of(90);
        public static final AngularAcceleration MAX_ACCEL_PROFILED = DegreesPerSecondPerSecond.of(45);

        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(kS, kV, kA);

        public static final SmartMotorControllerConfig MOTOR_CONFIG = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(kP, kI, kD, MAX_VEL_PROFILED, MAX_ACCEL_PROFILED)
                .withFeedforward(FEEDFORWARD)
                .withGearing(GEARING)
                .withIdleMode(MotorMode.COAST)
                .withStatorCurrentLimit(STATOR_LIMIT)
                .withClosedLoopRampRate(RAMP_RATE)
                .withOpenLoopRampRate(RAMP_RATE)
                .withFollowers(new Pair<>(FOLLOWER_ID, true))
                .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW);

        public static final FlyWheelConfig FLYWHEEL_CONFIG = new FlyWheelConfig()
                .withDiameter(WHEEL_DIAMETER)
                .withMass(WHEEL_MASS)
                .withLowerSoftLimit(MIN_VELOCITY)
                .withUpperSoftLimit(MAX_VELOCITY)
                .withTelemetry("Shooter", TelemetryVerbosity.LOW);
    }

public static class HoodConstants {
    public static final int MOTOR_ID = 27;
    public static final DCMotor MOTOR_TYPE = DCMotor.getKrakenX44(1);

    public static final MechanismGearing GEARING = new MechanismGearing(
            GearBox.fromReductionStages((24.0 / 12.0), (160.0 / 10.0)));
    public static final Angle STARTING_POS = Degrees.of(0);
    public static final Angle MIN_ANGLE = Degrees.of(0);
    public static final Angle MAX_ANGLE = Degrees.of(35);
    public static final Angle TOLERANCE = Degrees.of(1.0);

    public static final double kP = 4.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(10);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(5);
    public static final Current STATOR_LIMIT = Amps.of(40);
    public static final Time RAMP_RATE = Seconds.of(0.25);

    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    public static final SmartMotorControllerConfig MOTOR_CONFIG = new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(kP, kI, kD, MAX_VELOCITY, MAX_ACCELERATION)
            .withFeedforward(FEEDFORWARD)
            .withGearing(GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(STATOR_LIMIT)
            .withClosedLoopRampRate(RAMP_RATE)
            .withTelemetry("HoodMotor", TelemetryVerbosity.LOW);

    public static final PivotConfig HOOD_CONFIG = new PivotConfig()
            .withStartingPosition(STARTING_POS)
            .withHardLimit(MIN_ANGLE, MAX_ANGLE)
            .withSoftLimits(MIN_ANGLE, MAX_ANGLE)
            .withTelemetry("Hood", TelemetryVerbosity.LOW);
}

public static class TurretConstants {
        public static final int MOTOR_ID = 28;
        public static final DCMotor MOTOR_TYPE = DCMotor.getKrakenX44(1);

        public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(100));
        public static final Angle STARTING_POS = Degrees.of(0);
        public static final Angle MIN_ANGLE = Degrees.of(-180);
        public static final Angle MAX_ANGLE = Degrees.of(180);
        public static final Angle TOLERANCE = Degrees.of(2.0);

        public static final double kP = 2.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final AngularVelocity MAX_VEL = DegreesPerSecond.of(180);
        public static final AngularAcceleration MAX_ACCEL = DegreesPerSecondPerSecond.of(90);
        public static final Current STATOR_LIMIT = Amps.of(40);
        public static final Time RAMP_RATE = Seconds.of(0.25);

        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(kS, kV, kA);

        public static final SmartMotorControllerConfig MOTOR_CONFIG = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(kP, kI, kD, MAX_VEL, MAX_ACCEL)
                .withFeedforward(FEEDFORWARD)
                .withGearing(GEARING)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(STATOR_LIMIT)
                .withClosedLoopRampRate(RAMP_RATE)
                .withTelemetry("TurretMotor", TelemetryVerbosity.LOW);

        public static final PivotConfig TURRET_CONFIG = new PivotConfig()
                .withStartingPosition(STARTING_POS)
                .withHardLimit(MIN_ANGLE, MAX_ANGLE)
                .withSoftLimits(MIN_ANGLE, MAX_ANGLE)
                .withTelemetry("Turret", TelemetryVerbosity.LOW);
    }
}