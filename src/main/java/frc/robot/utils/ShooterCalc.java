package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants;
import frc.robot.utils.FieldConstants.Hub;
import frc.robot.utils.FieldConstants.LeftBump;
import frc.robot.utils.FieldConstants.LinesVertical;
import frc.robot.utils.FieldConstants.RightBump;
import frc.robot.utils.FieldConstants.Tower;
import frc.robot.utils.Liana.LianaHelpers;

public class ShooterCalc {

    private StructPublisher<Translation2d> targetPublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("ShooterCalc/Target", Translation2d.struct)
            .publish();

    private StructArrayPublisher<Pose3d> actualPath =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("ShooterCalc/ActualPath", Pose3d.struct)
            .publish();

    private StructArrayPublisher<Pose3d> targetedPath =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("ShooterCalc/TargetedPath", Pose3d.struct)
            .publish();

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

        Translation2d turretOffset = ShooterConstants.ROBOT_CENTER_TO_SHOOTER.toTranslation2d();
        double omega = state.Speeds.omegaRadiansPerSecond;

        double tanVelX = -omega * turretOffset.getY();
        double tanVelY = -omega * turretOffset.getX();

        Translation2d robotRelativeTurretVel = new Translation2d(
            state.Speeds.vxMetersPerSecond + tanVelX,
            state.Speeds.vyMetersPerSecond + tanVelY
        );

        Translation2d robotVel = robotRelativeTurretVel.rotateBy(pose.getRotation());

        Translation2d latencyOffset = robotVel.times(LATENCY);
        Translation2d turretPos = pose.getTranslation().plus(latencyOffset);

        Translation2d target = getTarget(pose);

        Translation2d lookaheadPos = turretPos;
        double lookaheadDist = target.getDistance(turretPos);

        for (int i = 0; i < 20; i++) {
            double tof = SHOOTER_MAP.get(lookaheadDist).tof();
            Translation2d offset = robotVel.times(tof);
            lookaheadPos = turretPos.plus(offset);
            lookaheadDist = target.getDistance(lookaheadPos);
        }

        Translation2d toGoal = target.minus(lookaheadPos);
        Rotation2d turretAngle = toGoal.getAngle().minus(pose.getRotation());

        FullShooterParams params = SHOOTER_MAP.get(lookaheadDist);

        double finalHoodAngle =
            params.hood() + LianaHelpers.getHoodAngleAdjustment();
        double vLaunchH = toGoal.getNorm() / params.tof();
        double vLaunchZ =
            vLaunchH * Math.tan(Degrees.of(finalHoodAngle - 15).in(Radians));

        Translation3d startPose3d = new Translation3d(
            pose.getX(),
            pose.getY(),
            ShooterConstants.ROBOT_CENTER_TO_SHOOTER.getZ()
        );

        Translation2d shotVelH = toGoal.div(toGoal.getNorm()).times(vLaunchH);
        Translation2d actualVelH = shotVelH.plus(robotVel);

        actualPath.set(
            buildPath(startPose3d, actualVelH, vLaunchZ, params.tof())
        );
        targetedPath.set(
            buildPath(startPose3d, shotVelH, vLaunchZ, params.tof())
        );

        return new ShooterSolution(
            Degrees.of(
                turretAngle.getDegrees() +
                    LianaHelpers.getTurretAngleAdjustment()
            ),
            Degrees.of(params.hood() + LianaHelpers.getHoodAngleAdjustment()),
            RPM.of(params.rpm() + LianaHelpers.getFlywheelAdjustment())
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
            double distToLeft = logicalPose
                .getTranslation()
                .getDistance(LeftBump.leftBumpTarget);
            double distToRight = logicalPose
                .getTranslation()
                .getDistance(RightBump.rightBumpTarget);

            double tarX = Tower.frontFaceX;
            double tarY = logicalPose.getY();
            targetLogical = new Translation2d(tarX, tarY);
        }
        var ret = AllianceFlip.apply(targetLogical);
        targetPublisher.set(ret);
        return ret;
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
}
