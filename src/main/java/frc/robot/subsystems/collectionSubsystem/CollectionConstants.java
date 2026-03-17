package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.*;

public final class CollectionConstants {

    public static final class IntakeRollerConstants {
        public static final int MOTOR_ID = 33;

        // INTAKE ROLLER SPEEDS
        public static final double INTAKE_SPEED = 0.65;
        public static final double OUTTAKE_SPEED = -0.25;

        public static final Current STATOR_LIMIT = Amps.of(60);
        public static final Current SUPPLY_LIMIT = Amps.of(40);
    }

    public static final class IntakeExtensionConstants {
        public static final int MOTOR_ID = 32;

        // GEARING
        public static final double GEAR_RATIO = 12.0;
        // TODO: Verify this value for the rack and pinion circumference
        public static final Distance INCHES_PER_ROT = Inches.of(1.0);

        public static final Distance STOW_POS = Inches.zero();
        public static final Distance INTAKE_POS = Inches.of(4.09);

        // MOTION MAGIC
        public static final double kP = 100;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final AngularVelocity MM_VELOCITY = RotationsPerSecond.of(1);
        public static final AngularAcceleration MM_ACCELERATION = RotationsPerSecondPerSecond.of(2);
        public static final Velocity<AngularAccelerationUnit> JERK =
            RotationsPerSecondPerSecond.per(Second).of(100);

        public static final Current STATOR_LIMIT = Amps.of(50);
        public static final Current SUPPLY_LIMIT = Amps.of(40);
    }

    public static final class IndexerConstants {
        public static final int MOTOR_ID = 44;

        // INDEXER SPEEDS
        public static final double FEED_SPEED = 0.25;
        public static final double OUTTAKE_SPEED = -0.25;

        public static final Current STATOR_LIMIT = Amps.of(35);
    }

    public static final class KickerConstants {
        public static final int LEADER_ID = 45;
        public static final int FOLLOWER_ID = 46;

        // KICKER SPEEDS
        public static final double FEED_SPEED = 0.35;
        public static final double OUTTAKE_SPEED = -0.25;

        public static final Current STATOR_LIMIT = Amps.of(35);
    }
}
