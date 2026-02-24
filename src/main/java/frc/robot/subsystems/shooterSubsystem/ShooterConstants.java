package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class ShooterConstants {

    public static final Translation3d ROBOT_CENTER_TO_SHOOTER =
        new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0));

    public static class FlyWheelConstants {

        public static final int LEADER_ID = 28;
        public static final int FOLLOWER_ID = 29;

        public static final DCMotor MOTORS = DCMotor.getKrakenX60Foc(2);
        public static final Distance WHEEL_DIAMETER = Inches.of(4);
        public static final Mass WHEEL_MASS = Pounds.of(5);
        public static final MechanismGearing GEARING = new MechanismGearing(1);

        public static final Current STATOR_LIMIT = Amps.of(40);
        public static final AngularVelocity MIN_VELOCITY = RPM.of(0);
        public static final AngularVelocity MAX_VELOCITY = RPM.of(6000);
        public static final AngularVelocity TOLERANCE = RPM.of(90);
        public static final Time RAMP_RATE = Seconds.of(0.25);

        public static final double kP = 25.0;
        public static final double kI = 0.0;
        public static final double kD = 75.0;
    }

    public static class HoodConstants {

        public static final int MOTOR_ID = 31;
        public static final DCMotor MOTOR_TYPE = DCMotor.getKrakenX44(1);

        public static final Distance MOI_LENGTH = Inches.of(6);
        public static final Mass MOI_MASS = Pounds.of(2);

        public static final MechanismGearing GEARING = new MechanismGearing(
            GearBox.fromReductionStages((24.0 / 12.0), (160.0 / 10.0))
        );
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
        public static final Current STATOR_LIMIT = Amps.of(40);
    }

    public static class TurretConstants {

        public static final int ENCODER_13_ID = 50;
        public static final int ENCODER_14_ID = 51;
        public static final int MOTOR_ID = 32;
        public static final DCMotor MOTOR_TYPE = DCMotor.getKrakenX44(1);

        public static final Distance LENGTH = Meters.of(0.25);
        public static final Mass WEIGHT = Pounds.of(10);
        public static final MechanismGearing GEARING = new MechanismGearing(
            GearBox.fromReductionStages(32)
        );
        public static final Angle STARTING_POS = Degrees.of(0);
        public static final Angle MIN_ANGLE = Degrees.of(0);
        public static final Angle MAX_ANGLE = Degrees.of(355);
        public static final Angle TOLERANCE = Degrees.of(0.5);

        public static final double kP = 325.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;


        public static final AngularVelocity MAX_VEL = DegreesPerSecond.of(900);
        public static final AngularAcceleration MAX_ACCEL =
            DegreesPerSecondPerSecond.of(1800);
        public static final Current STATOR_LIMIT = Amps.of(60);

        public static final Angle ENCODER_14_OFFSET = Rotations.of(0);
        public static final Angle ENCODER_13_OFFSET = Rotations.of(0);
    }
}
