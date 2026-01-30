package frc.robot.utils;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.FieldConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

public class ShooterCalc {

    public record FullShooterParams(double rpm, double hood, double tof) {
        public static FullShooterParams interpolate(
            FullShooterParams s,
            FullShooterParams e,
            double t
        ) {
            return new FullShooterParams(
                MathUtil.interpolate(s.rpm, e.rpm, t),
                MathUtil.interpolate(s.hood, e.hood, t),
                MathUtil.interpolate(s.tof, e.tof, t)
            );
        }
    }

    public record ShooterPosition(Angle hood, AngularVelocity flywheel) {}

    private record Solution(Rotation2d angle, FullShooterParams params) {}

    private static final InterpolatingTreeMap<
        Double,
        FullShooterParams
    > SHOOTER_MAP = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(),
        FullShooterParams::interpolate
    );
    private static final InterpolatingDoubleTreeMap REVERSE_MAP =
        new InterpolatingDoubleTreeMap();

    static {
        //Distance (m), RPM, Hood (deg), Time (s)
        add(1.5, 2800, 35, 0.38);
        add(2.0, 3100, 38, 0.45);
        add(2.5, 3400, 42, 0.52);
        add(3.0, 3650, 46, 0.60);
        add(3.5, 3900, 50, 0.68);
        add(4.0, 4100, 54, 0.76);
        add(4.5, 4350, 58, 0.85);
        add(5.0, 4550, 62, 0.94);
        add(6.0, 5000, 65, 1.10);
    }

    private static void add(double dist, double rpm, double hood, double tof) {
        SHOOTER_MAP.put(dist, new FullShooterParams(rpm, hood, tof));
        if (tof > 0) REVERSE_MAP.put(dist / tof, dist);
    }

    private final CommandSwerveDrivetrain drive;
    private static final double LATENCY = 0.06;

    public ShooterCalc(CommandSwerveDrivetrain drive) {
        this.drive = drive;
    }

    private Translation2d getTarget(Pose2d pose) {
        Pose2d logicalPose = AllianceFlip.apply(pose);

        Translation2d targetLogical;

        if (logicalPose.getX() < LinesVertical.allianceZone) {
            targetLogical = Hub.innerCenterPoint.toTranslation2d();
        } else {
            targetLogical = (logicalPose.getY() > LinesHorizontal.center)
                ? LeftBump.nearLeftCorner
                : RightBump.nearRightCorner;
        }

        return AllianceFlip.apply(targetLogical);
    }

    private Solution calculate() {
        var state = drive.getState();
        Pose2d pose = state.Pose;

        Translation2d robotVel = new Translation2d(
            state.Speeds.vxMetersPerSecond,
            state.Speeds.vyMetersPerSecond
        ).rotateBy(pose.getRotation());
        Translation2d futurePos = pose
            .getTranslation()
            .plus(robotVel.times(LATENCY));

        Translation2d toGoal = getTarget(pose).minus(futurePos);
        double dist = toGoal.getNorm();
        FullShooterParams baseline = SHOOTER_MAP.get(dist);

        Translation2d shotVel = toGoal
            .div(dist)
            .times(dist / baseline.tof)
            .minus(robotVel);

        return new Solution(
            shotVel.getAngle().minus(pose.getRotation()),
            SHOOTER_MAP.get(REVERSE_MAP.get(shotVel.getNorm()))
        );
    }

    public Supplier<Angle> getTurretAngleSupplier() {
        return () -> Degrees.of(calculate().angle.getDegrees());
    }

    public Supplier<ShooterPosition> getShooterPositionSupplier() {
        return () -> {
            var sol = calculate();
            return new ShooterPosition(
                Degrees.of(sol.params.hood),
                RPM.of(sol.params.rpm)
            );
        };
    }
}
