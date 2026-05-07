package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;

public class ShooterConstants {

    public static final Transform3d ROBOT_CENTER_TO_SHOOTER =
        new Transform3d(
            new Translation3d(Inches.of(-5.375), Inches.of(-1.25), Inches.of(12.385)),
            new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(90)));

    public static class FlyWheelConstants {

        public static final int LEADER_ID = 28;
        public static final int FOLLOWER_ID = 29;

        public static final double GEAR_RATIO = 1.0;

        public static final Current STATOR_LIMIT = Amps.of(100);
        public static final Current SUPPLY_LIMIT = Amps.of(60);
        public static final AngularVelocity MIN_VELOCITY = RPM.of(0);
        public static final AngularVelocity MAX_VELOCITY = RPM.of(5500);
        public static final AngularVelocity TOLERANCE = RPM.of(200);

        public static final double P = 4.5;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double S = 3.75;
        public static final double V = 0.013;
    }

    public static class HoodConstants {

        public static final int MOTOR_ID = 31;

        public static final Distance MOI_LENGTH = Inches.of(6);
        public static final Mass MOI_MASS = Pounds.of(2);

        public static final double GEAR_RATIO = (24.0 / 12.0) * (160.0 / 10.0);
        public static final Angle STARTING_POS = Degrees.of(0);
        public static final Angle MIN_ANGLE = Degrees.of(0);
        public static final Angle MAX_ANGLE = Degrees.of(19.5);
        public static final Angle TOLERANCE = Degrees.of(1.0);

        public static final double kP = 900.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(
            900
        );
        public static final AngularAcceleration MAX_ACCELERATION =
            DegreesPerSecondPerSecond.of(1800);
        public static final Current STATOR_LIMIT = Amps.of(20);
    }

    public static class TurretConstants {

        public static final int ENCODER_13_ID = 50;
        public static final int ENCODER_14_ID = 51;
        public static final int MOTOR_ID = 30;

        public static final Distance LENGTH = Inches.one();
        public static final Mass WEIGHT = Pounds.of(10);
        public static final double GEAR_RATIO = 32.0;
        public static final Angle STARTING_POS = Degrees.of(0);
        public static final Angle MIN_ANGLE = Degrees.of(-10);
        public static final Angle MAX_ANGLE = Degrees.of(200);
        public static final Angle TOLERANCE = Degrees.of(0.5);

        public static final double kP = 325.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final AngularVelocity MAX_VEL = DegreesPerSecond.of(900);
        public static final AngularAcceleration MAX_ACCEL =
            DegreesPerSecondPerSecond.of(1800);
        public static final Current STATOR_LIMIT = Amps.of(40);

        public static final Angle ENCODER_14_OFFSET = Rotations.of(-0.044);
        public static final Angle ENCODER_13_OFFSET = Rotations.of(-0.321);
        public static final double ENCODER_13_RATIO = 80.0 / 13.0;
        public static final double ENCODER_14_RATIO = 80.0 / 14.0;
        public static final Angle ENCODER_MATCH_TOLERANCE = Rotations.of(0.06);
        public static final boolean ENCODER_13_INVERTED = true;
        public static final boolean ENCODER_14_INVERTED = true;
    }
}
