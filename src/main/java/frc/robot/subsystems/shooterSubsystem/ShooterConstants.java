package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.*;

public final class ShooterConstants {

    public static final Translation2d TURRET_MOUNT_OFFSET =
        new Translation2d(Inches.of(-5.375), Inches.of(-1.25));

    public static final Rotation2d TURRET_ZERO_HEADING = Rotation2d.fromDegrees(90);

    public static final class FlyWheelConstants {

        public static final int LEADER_ID = 28;
        public static final int FOLLOWER_ID = 29;

        public static final double GEAR_RATIO = 1.0;

        public static final Current STATOR_LIMIT = Amps.of(100);
        public static final Current SUPPLY_LIMIT = Amps.of(60);
        public static final AngularVelocity MIN_VELOCITY = RPM.of(0);
        public static final AngularVelocity MAX_VELOCITY = RPM.of(5500);
        public static final AngularVelocity TOLERANCE = RPM.of(200);

        public static final double kP = 4.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 3.75;
        public static final double kV = 0.013;
    }

    public static final class HoodConstants {

        public static final int MOTOR_ID = 31;

        public static final double GEAR_RATIO = (24.0 / 12.0) * (160.0 / 10.0);

        public static final Angle STARTING_POS = Degrees.of(0);
        public static final Angle MIN_ANGLE = Degrees.of(0);
        public static final Angle MAX_ANGLE = Degrees.of(19.5);
        public static final Angle TOLERANCE = Degrees.of(1.0);

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;

        public static final AngularVelocity MM_VELOCITY =
            DegreesPerSecond.of(20);
        public static final AngularAcceleration MM_ACCELERATION =
            DegreesPerSecondPerSecond.of(40);
        public static final Velocity<AngularAccelerationUnit> JERK =
            RotationsPerSecondPerSecond.per(Second).of(100);

        public static final Current STATOR_LIMIT = Amps.of(20);
        public static final Current SUPPLY_LIMIT = Amps.of(5);
    }

    public static final class TurretConstants {

        public static final int ENCODER_13_ID = 50;
        public static final int ENCODER_14_ID = 51;
        public static final int MOTOR_ID = 30;

        public static final double GEAR_RATIO = 32.0;

        public static final Angle STARTING_POS = Degrees.of(0);
        public static final Angle MIN_ANGLE = Degrees.of(-10);
        public static final Angle MAX_ANGLE = Degrees.of(200);
        public static final Angle TOLERANCE = Degrees.of(0.5);

        public static final double kP = 5.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;

        public static final AngularVelocity MM_VELOCITY = DegreesPerSecond.of(100);
        public static final AngularAcceleration MM_ACCELERATION =
            DegreesPerSecondPerSecond.of(200);
        public static final Velocity<AngularAccelerationUnit> JERK =
            RotationsPerSecondPerSecond.per(Second).of(100);

        public static final Current STATOR_LIMIT = Amps.of(40);
        public static final Current SUPPLY_LIMIT = Amps.of(20);

        public static final Angle ENCODER_14_OFFSET = Rotations.of(-0.044);
        public static final Angle ENCODER_13_OFFSET = Rotations.of(-0.321);
    }
}
