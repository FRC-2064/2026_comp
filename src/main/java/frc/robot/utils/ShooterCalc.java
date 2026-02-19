package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants;
import frc.robot.utils.FieldConstants.Hub;
import frc.robot.utils.FieldConstants.LeftBump;
import frc.robot.utils.FieldConstants.LinesVertical;
import frc.robot.utils.FieldConstants.RightBump;

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

    public record ShooterSolution(
        Angle turretAngle,
        Angle hoodAngle,
        AngularVelocity flywheelVelocity
    ) {}

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

    public ShooterSolution getSelectedSolution() {
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
        double dist = Math.max(toGoal.getNorm(), 0.001);

        FullShooterParams baseline = SHOOTER_MAP.get(dist);

        Translation2d shotVel = toGoal
            .div(dist)
            .times(dist / baseline.tof)
            .minus(robotVel);

        Rotation2d baseTurretAngle = shotVel
            .getAngle()
            .minus(pose.getRotation());
        FullShooterParams params = SHOOTER_MAP.get(
            REVERSE_MAP.get(shotVel.getNorm())
        );

        double turretAdj = LianaHelpers.getTurretAngleAdjustment();
        double hoodAdj = LianaHelpers.getHoodAngleAdjustment();
        double flywheelAdj = LianaHelpers.getFlywheelAdjustment();

        double finalHoodAngle = params.hood + hoodAdj;
        double vLaunchH = shotVel.getNorm();
        double vLaunchZ =
            vLaunchH * Math.tan(Degrees.of(finalHoodAngle - 15).in(Radians));

        Translation3d startPose = new Translation3d(
            pose.getX(),
            pose.getY(),
            ShooterConstants.ROBOT_CENTER_TO_SHOOTER.getZ()
        );
        Translation2d actualVelH = shotVel.plus(robotVel);

        actualPath.set(
            buildPath(startPose, actualVelH, vLaunchZ, baseline.tof())
        );
        targetedPath.set(
            buildPath(startPose, shotVel, vLaunchZ, baseline.tof())
        );

        return new ShooterSolution(
            Degrees.of(baseTurretAngle.getDegrees() + turretAdj),
            Degrees.of(params.hood + hoodAdj),
            RPM.of(params.rpm + flywheelAdj)
        );
    }

    private Translation2d getTarget(Pose2d pose) {
        Pose2d logicalPose = AllianceFlip.apply(pose);
        Translation2d targetLogical;

        double halfSize = Inches.of(13.75).in(Meters);
        double bumperOffset =
            halfSize * Math.abs(logicalPose.getRotation().getCos()) +
            halfSize * Math.abs(logicalPose.getRotation().getSin());

        double minX = logicalPose.getX() - bumperOffset;

        if (minX < LinesVertical.allianceZone) {
            targetLogical = Hub.innerCenterPoint.toTranslation2d();
        } else {
            double distToLeft = logicalPose.getTranslation().getDistance(LeftBump.leftBumpTarget);
            double distToRight = logicalPose.getTranslation().getDistance(RightBump.rightBumpTarget);
            targetLogical = (distToLeft < distToRight) ? LeftBump.leftBumpTarget : RightBump.rightBumpTarget;
        }
        return AllianceFlip.apply(targetLogical);
    }

    private Pose3d[] buildPath(
        Translation3d start,
        Translation2d velH,
        double velZ,
        double tof
    ) {
        int steps = 20;
        Pose3d[] path = new Pose3d[steps];
        double dt = tof / (steps - 1);

        for (int i = 0; i < steps; i++) {
            double t = i * dt;
            double x = start.getX() + velH.getX() * t;
            double y = start.getY() + velH.getY() * t;
            double z = start.getZ() + (velZ * t) - (0.5 * 9.81 * t * t);
            path[i] = new Pose3d(x, y, z, new Rotation3d());
        }
        return path;
    }

    private void publishTranslation(Translation2d loc) {
        publisher.set(new Translation2d[] { loc });
    }
}
