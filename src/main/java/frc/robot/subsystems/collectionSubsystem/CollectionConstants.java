package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class CollectionConstants {

    public static class IntakeRollerConstants {
        public static final int ROLLER_ID = 33;

        // INTAKE ROLLER SPEEDS
        public static final double INTAKE = 0.50;
        public static final double OUTTAKE = -0.25;
        public static final Current STATOR_LIMIT = Amps.of(60);
    }

    public static class IntakeConstants {
        public static final int EXTEND_ID = 32;

        // GEARING
        public static final double RACK_GEARING = 12.0;

        // 14T @ 10DP: PD of 1.4in
        public static final Distance INCHES_PER_ROT = Inches.of(Math.PI * 1.4);
        public static final Angle STOW = Rotations.zero();
        public static final Angle INTAKE = Rotations.of(9.1 / INCHES_PER_ROT.in(Inches));
        public static final Angle AGITATE = INTAKE.div(2);

        // MOTION MAGIC
        public static final double P = 50;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final LinearVelocity MAX_VEL = InchesPerSecond.of(4.0);
        public static final LinearAcceleration MAX_ACCEL = InchesPerSecondPerSecond.of(8.0);
        public static final AngularVelocity MM_CRUISE_VEL = RotationsPerSecond.of(MAX_VEL.in(InchesPerSecond) / INCHES_PER_ROT.in(Inches));
        public static final AngularAcceleration MM_ACCEL = RotationsPerSecondPerSecond.of(MAX_ACCEL.in(InchesPerSecondPerSecond) / INCHES_PER_ROT.in(Inches));


        public static final Current STATOR_LIMIT = Amps.of(40);
    }

    public static class IndexerConstants {
        public static final int MOTOR_ID = 44;

        // SPINDEXER SPEEDS
        public static final double FEED = 0.75;
        public static final double OUTTAKE = -0.25;
        public static final Current STATOR_LIMIT = Amps.of(40);
    }

    public static class KickerConstants {

        public static final int KICKER_LEADER_ID = 45;
        public static final int KICKER_FOLLOWER_ID = 46;

        // KICKER SPEEDS
        public static final double FEED = 0.50;
        public static final double OUTTAKE = -0.25;
        public static final int CURRENT_LIMIT = 40;
    }
}
