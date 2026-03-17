package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import frc.robot.utils.FieldConstants.Hub;
import frc.robot.utils.FieldConstants.LinesVertical;
import frc.robot.utils.FieldConstants.Tower;
import frc.robot.utils.Liana.LianaHelpers;

public class ShooterCalc {

    private StructPublisher<Translation2d> targetPublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("ShooterCalc/Target", Translation2d.struct)
            .publish();

    private StructPublisher<Translation2d> targetShot =
        NetworkTableInstance.getDefault()
            .getStructTopic("ShooterCalc/TargetShot", Translation2d.struct)
            .publish();

    private StructPublisher<Translation2d> turretPosPublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("ShooterCalc/TurretPos", Translation2d.struct)
            .publish();

    private StructPublisher<Translation2d> targetPublisher2 =
        NetworkTableInstance.getDefault()
            .getStructTopic("ShooterCalc/LookaheadPos", Translation2d.struct)
            .publish();

    private StructPublisher<Transform3d> transformPub = NetworkTableInstance.getDefault()
        .getStructTopic("shootercalc/turretLoc", Transform3d.struct)
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
        add(3.39, 5000, 8, 0);
        add(4.37, 5250, 10, 0);
        add(3.15, 4500, 8, 0);
    }

    private static void add(double dist, double rpm, double hood, double tof) {
        SHOOTER_MAP.put(dist, new FullShooterParams(rpm, hood, tof));
        if (tof > 0) REVERSE_MAP.put(dist / tof, dist);
    }

    private final CommandSwerveDrivetrain drive;
    private static final double LATENCY = 0.06;

    public ShooterCalc(CommandSwerveDrivetrain drive) {
        this.drive = drive;

        transformPub.set(ShooterConstants.ROBOT_CENTER_TO_SHOOTER);
    }

    public ShooterSolution getSelectedSolution() {
        var state = drive.getState();
        Pose2d pose = state.Pose;

        var turretZeroInFieldFrame = pose.getRotation()
            .plus(new Rotation2d(
                ShooterConstants.ROBOT_CENTER_TO_SHOOTER.getRotation().getZ()
            ));

        var turretMountOffset = ShooterConstants.ROBOT_CENTER_TO_SHOOTER
            .getTranslation()
            .toTranslation2d()
            .rotateBy(pose.getRotation());

        double omega = state.Speeds.omegaRadiansPerSecond;
        double tanVelX = -omega * ShooterConstants.ROBOT_CENTER_TO_SHOOTER.getTranslation().getY();
        double tanVelY =  omega * ShooterConstants.ROBOT_CENTER_TO_SHOOTER.getTranslation().getX();

        Translation2d robotRelativeTurretVel = new Translation2d(
            state.Speeds.vxMetersPerSecond + tanVelX,
            state.Speeds.vyMetersPerSecond + tanVelY
        );

        double speed = Math.hypot(
            state.Speeds.vxMetersPerSecond,
            state.Speeds.vyMetersPerSecond
        );
        Translation2d robotVel = speed > 0.05
            ? robotRelativeTurretVel.rotateBy(pose.getRotation())
            : Translation2d.kZero;

        Translation2d latencyOffset = robotVel.times(LATENCY);
        Translation2d turretPos = pose.getTranslation()
            .plus(turretMountOffset)
            .plus(latencyOffset);

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
        Rotation2d turretAngle = toGoal.getAngle().minus(turretZeroInFieldFrame);

        FullShooterParams params = SHOOTER_MAP.get(lookaheadDist);

        targetShot.set(toGoal);
        turretPosPublisher.set(turretPos);
        targetPublisher2.set(lookaheadPos);

        return new ShooterSolution(
            Degrees.of(
                MathUtil.clamp(
                    turretAngle.getDegrees() + LianaHelpers.getTurretAngleAdjustment(),
                    TurretConstants.MIN_ANGLE.in(Degrees),
                    TurretConstants.MAX_ANGLE.in(Degrees)
                )
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

            double tarX = Tower.frontFaceX;
            double tarY = logicalPose.getY();
            targetLogical = new Translation2d(tarX, tarY);
        }
        var ret = AllianceFlip.apply(targetLogical);
        targetPublisher.set(ret);
        return ret;
    }

}
