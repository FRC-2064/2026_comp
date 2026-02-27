package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class RobotConstants {

    public static TelemetryVerbosity GetTelemetry() {
        if (RobotBase.isSimulation()) {
            return TelemetryVerbosity.HIGH;
        }
        return TelemetryVerbosity.LOW;
    }

    public static class DriveConstants {

        public static final LinearVelocity MAX_DRIVE_SPEED = MetersPerSecond.of(
            5.23
        );
        public static final AngularVelocity MAX_ROT_SPEED = RadiansPerSecond.of(
            Math.PI * 1.5
        );

        public static final LinearVelocity FAST_DRIVE_SPEED =
            MAX_DRIVE_SPEED.times(1.5);
        public static final AngularVelocity FAST_ROT_SPEED =
            MAX_ROT_SPEED.times(1.5);

        public static final LinearAcceleration MAX_ACCEL =
            MetersPerSecondPerSecond.of(3.0);

        public static final double DEADBAND = 0.1;
    }

    public static class SuperstructureConstants {

        public static final double READY_TO_SHEET_DEBOUNCE_SECONDS = 0.25;

        public static final double STOW_SPEED = 1.0;
        public static final double OUTTAKE_SPEED = 1.0;
        public static final double INTAKE_SPEED = 0.75;
        public static final double SHOOT_SPEED = 0.75;
        public static final double SNOWBLOW_SPEED = 0.25;

        public static final double MANUAL_TURRET_DEADBAND = 0.1;
        public static final Angle MANUAL_TURRET_RATE = Degrees.of(3);
    }
}
