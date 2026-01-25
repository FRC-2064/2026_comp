package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class CollectionConstants {

    public static class IntakeConstants {
        public static final int WRIST_ID = 11;
        public static final int ROLLER_ID = 12;

        public static final DCMotor WRIST_MOTOR_TYPE = DCMotor.getKrakenX44(1);
        public static final MechanismGearing WRIST_GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4));
        public static final Distance MOI_RADIUS = Meters.of(0.25);
        public static final Mass MOI_MASS = Pounds.of(4);

        public static final double kP = 4.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final AngularVelocity MAX_VEL = DegreesPerSecond.of(180);
        public static final AngularAcceleration MAX_ACCEL = DegreesPerSecondPerSecond.of(90);
        public static final Current STATOR_LIMIT = Amps.of(40);

        public static final Angle STOW_ANGLE = Degrees.of(0);
        public static final Angle INTAKE_ANGLE = Degrees.of(90);
        public static final double INTAKE_SPEED = 1.0;
        public static final double OUTTAKE_SPEED = -1.0;

        public static final SmartMotorControllerConfig WRIST_MOTOR_CONFIG = new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(kP, kI, kD, MAX_VEL, MAX_ACCEL)
            .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
            .withGearing(WRIST_GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(STATOR_LIMIT)
            .withTelemetry("IntakeWristMotor", TelemetryVerbosity.LOW);

        public static final PivotConfig WRIST_CONFIG = new PivotConfig()
            .withStartingPosition(STOW_ANGLE)
            .withHardLimit(Degrees.of(0), Degrees.of(120))
            .withMOI(MOI_RADIUS, MOI_MASS)
            .withTelemetry("IntakeWrist", TelemetryVerbosity.LOW);
    }

    public static class IndexerConstants {
        public static final int MOTOR_ID = 13;
        public static final double FEED_SPEED = 0.9;
        public static final double OUTTAKE_SPEED = -0.9;
        public static final Current STATOR_LIMIT = Amps.of(40);
    }
}