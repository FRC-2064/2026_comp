package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public final class DriveConstants {

    public static final LinearVelocity MAX_DRIVE_SPEED = MetersPerSecond.of(
        5.23
    );
    public static final AngularVelocity MAX_ROT_SPEED = RadiansPerSecond.of(
        Math.PI * 1.5
    );

    public static final LinearVelocity FAST_DRIVE_SPEED = MAX_DRIVE_SPEED.times(
        1.5
    );
    public static final AngularVelocity FAST_ROT_SPEED = MAX_ROT_SPEED.times(
        1.5
    );

    public static final LinearAcceleration MAX_ACCEL =
        MetersPerSecondPerSecond.of(3.0);

    public static final double DEADBAND = 0.1;
}
