package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class CollectionConstants {

    public static class IntakeRollerConstants {
        public static final int ROLLER_ID = 33;

        public static final double INTAKE = 0.25;
        public static final double OUTTAKE = -0.25;
        public static final Current STATOR_LIMIT = Amps.of(60);
    }

    public static class IntakeConstants {
        public static final int EXTEND_ID = 32;

        public static final DCMotor MOTOR_TYPE = DCMotor.getKrakenX44(1);
        public static final MechanismGearing RACK_GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4));
        public static final Mass MOI_MASS = Pounds.of(4);
        public static final double kP = 1000;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final AngularVelocity MAX_VEL = DegreesPerSecond.of(180);
        public static final AngularAcceleration MAX_ACCEL = DegreesPerSecondPerSecond.of(90);
        public static final Current STATOR_LIMIT = Amps.of(40);

        public static final Distance STOW_HEIGHT = Inches.zero();
        public static final Distance INTAKE_HEIGHT = Inches.of(10.018);
    }

    public static class IndexerConstants {
        public static final int MOTOR_ID = 44;

        public static final double FEED = 0.9;
        public static final double OUTTAKE = -0.9;
        public static final Current STATOR_LIMIT = Amps.of(40);
    }

    public static class KickerConstants {

        public static final int KICKER_LEADER_ID = 45;
        public static final int KICKER_FOLLOWER_ID = 46;

        public static final double FEED = 0.75;
        public static final double OUTTAKE = -0.5;
        public static final Current CURRENT_LIMIT = Amps.of(40);
    }
}
