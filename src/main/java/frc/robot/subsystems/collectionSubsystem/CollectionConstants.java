package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class CollectionConstants {

    public static class IntakeConstants {
        public static final int EXTEND_ID = 32;
        public static final int ROLLER_ID = 33;

        public static final DCMotor RACK_MOTOR_TYPE = DCMotor.getKrakenX44(1);
        public static final MechanismGearing RACK_GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4));
        public static final Distance MOI_RADIUS = Meters.of(0.25);
        public static final Mass MOI_MASS = Pounds.of(4);
        public static final DCMotor MOTOR_TYPE = DCMotor.getKrakenX60Foc(1);

        public static final double kP = 4.0;
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

        public static final Distance STOW_HEIGHT = Meters.of(0);
        public static final Distance INTAKE_HEIGHT = Inches.of(10.018);
        public static final double INTAKE_SPEED = 1.0;
        public static final double OUTTAKE_SPEED = -1.0;
    }

    public static class IndexerConstants {
        public static final int MOTOR_ID = 13;
        public static final double FEED_SPEED = 0.9;
        public static final double OUTTAKE_SPEED = -0.9;
        public static final Current STATOR_LIMIT = Amps.of(40);
    }
}