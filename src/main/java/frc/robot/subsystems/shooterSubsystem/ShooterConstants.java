package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.*;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
        public static final Mass WHEEL_MASS = Pounds.of(4.15);
        public static final MechanismGearing GEARING = new MechanismGearing(1);

        public static final Current STATOR_LIMIT = Amps.of(40);
        public static final AngularVelocity MIN_VELOCITY = RPM.of(0);
        public static final AngularVelocity MAX_VELOCITY = RPM.of(6000);
        public static final AngularVelocity TOLERANCE = RPM.of(50);
        public static final Time RAMP_RATE = Seconds.of(0.25);

        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.05;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        public static final AngularVelocity MAX_VEL_PROFILED =
            DegreesPerSecond.of(90);
        public static final AngularAcceleration MAX_ACCEL_PROFILED =
            DegreesPerSecondPerSecond.of(45);

        public static final SimpleMotorFeedforward FEEDFORWARD =
            new SimpleMotorFeedforward(kS, kV, kA);
    }

    public static class HoodConstants {

        public static final Voltage HOMING_VOLTAGE = Volts.of(-1.5);
        public static final Current HOMING_STALL_CURRENT = Amps.of(8);
        public static final Time HOMING_STALL_TIME = Milliseconds.of(200);
        public static final Time HOMING_TIMEOUT = Seconds.of(5);
        public static final Angle HOMING_OFFSET = Degrees.of(0);

        public static final int MOTOR_ID = 31;
        public static final DCMotor MOTOR_TYPE = DCMotor.getKrakenX44(1);

        public static final Distance MOI_LENGTH = Inches.of(6);
        public static final Mass MOI_MASS = Pounds.of(2);

        public static final MechanismGearing GEARING = new MechanismGearing(
            GearBox.fromReductionStages((24.0 / 12.0), (160.0 / 10.0))
        );
        public static final Angle STARTING_POS = Degrees.of(0);
        public static final Angle MIN_ANGLE = Degrees.of(0);
        public static final Angle MAX_ANGLE = Degrees.of(35);
        public static final Angle TOLERANCE = Degrees.of(1.0);

        public static final double kP = 6.0;
        public static final double kI = 0.0;
        public static final double kD = 0.2;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(
            10
        );
        public static final AngularAcceleration MAX_ACCELERATION =
            DegreesPerSecondPerSecond.of(5);
        public static final Current STATOR_LIMIT = Amps.of(40);
        public static final Time RAMP_RATE = Seconds.of(0.25);

        public static final SimpleMotorFeedforward FEEDFORWARD =
            new SimpleMotorFeedforward(kS, kV, kA);
    }

    public static class TurretConstants {

        public static final int MOTOR_ID = 30;
        public static final DCMotor MOTOR_TYPE = DCMotor.getKrakenX44(1);

        public static final Distance LENGTH = Meters.of(0.25);
        public static final Mass WEIGHT = Pounds.of(10);
        public static final MechanismGearing GEARING = new MechanismGearing(
            GearBox.fromReductionStages(100)
        );
        public static final Angle STARTING_POS = Degrees.of(5);
        public static final Angle MIN_ANGLE = Degrees.of(0);
        public static final Angle MAX_ANGLE = Degrees.of(355);
        public static final Angle TOLERANCE = Degrees.of(2.0);

        public static final double kP = 50.0;
        public static final double kI = 0.0;
        public static final double kD = 2.0;
        public static final double kS = 0.0;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        public static final double kP_SIM = 50.0;
        public static final double kI_SIM = 0.0;
        public static final double kD_SIM = 2.0;
        public static final double kS_SIM = 0.0;
        public static final double kV_SIM = 0.12;
        public static final double kA_SIM = 0.0;

        public static final AngularVelocity MAX_VEL = DegreesPerSecond.of(180);
        public static final AngularAcceleration MAX_ACCEL =
            DegreesPerSecondPerSecond.of(360);
        public static final Current STATOR_LIMIT = Amps.of(40);
        public static final Time RAMP_RATE = Seconds.of(0.25);

        public static final SimpleMotorFeedforward FEEDFORWARD =
            new SimpleMotorFeedforward(kS, kV, kA);

            public static final SimpleMotorFeedforward FEEDFORWARD_SIM =
            new SimpleMotorFeedforward(kS_SIM, kV_SIM, kA_SIM);
    }
}
