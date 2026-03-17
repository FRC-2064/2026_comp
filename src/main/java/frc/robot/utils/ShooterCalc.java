package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import frc.robot.utils.FieldConstants.Hub;
import frc.robot.utils.FieldConstants.LinesVertical;
import frc.robot.utils.FieldConstants.Tower;
import frc.robot.utils.Liana.LianaHelpers;

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
    ) {
        public static ShooterSolution zero() {
            return new ShooterSolution(Degrees.zero(), Degrees.zero(), RPM.zero());
        }

        public ShooterSolution withTurretAngle(Angle turretAngle) {
            return new ShooterSolution(
                turretAngle,
                hoodAngle(),
                flywheelVelocity()
            );
        }
    }

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

    private static final double LATENCY = 0.06;


    public static Translation2d getTarget(Pose2d pose) {
        var logicalPose = AllianceFlip.apply(pose);
        Translation2d targetLogical;

        var halfSize = Inches.of(13.75).in(Meters);
        var bumperOffset =
        halfSize * Math.abs(logicalPose.getRotation().getCos()) +
        halfSize * Math.abs(logicalPose.getRotation().getSin());

        var minX = logicalPose.getX() - bumperOffset;

        if (minX < LinesVertical.ALLIANCE_ZONE) {
            targetLogical = Hub.INNER_CENTER_POINT.toTranslation2d();
        } else {
            var tarX = Tower.FRONT_FACE_X;
            var tarY = logicalPose.getY();
            targetLogical = new Translation2d(tarX, tarY);
        }

        return AllianceFlip.apply(targetLogical);

    }


    public static ShooterSolution getSelectedSolution(SwerveDriveState state) {
        var pose = state.Pose;

        var vx = state.Speeds.vxMetersPerSecond;
        var vy = state.Speeds.vyMetersPerSecond;
        var omega = state.Speeds.omegaRadiansPerSecond;

        // calc the pose after the latency
        var estimatedPose = pose.exp(
            new Twist2d(vx * LATENCY, vy * LATENCY, omega * LATENCY)
        );

        // transform the pose to the turret
        var turretPose = estimatedPose.transformBy(new Transform2d(ShooterConstants.TURRET_MOUNT_OFFSET, Rotation2d.kZero));

        // get field relative turret vel
        var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, estimatedPose.getRotation());

        // tangent vel of turret
        var turretTangentVelRobotRelative = new Translation2d(
            -omega * ShooterConstants.TURRET_MOUNT_OFFSET.getY(),
            -omega * ShooterConstants.TURRET_MOUNT_OFFSET.getX()
        );

        // rotate vel to field frame
        var turretTangentVelFieldRelative = turretTangentVelRobotRelative.rotateBy(estimatedPose.getRotation());
        var turretVel = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond)
            .plus(turretTangentVelFieldRelative);

        // account for robot vel
        var targetTrans = getTarget(estimatedPose);
        var lookaheadPose = turretPose;
        var lookaheadDist = targetTrans.getDistance(turretPose.getTranslation());

        for (int i = 0; i < 5; i++) {
            var tof = SHOOTER_MAP.get(lookaheadDist).tof();
            var offset = turretVel.times(tof);

            var newLookaheadTranslation = turretPose.getTranslation().plus(offset);

            if (newLookaheadTranslation.getDistance(lookaheadPose.getTranslation()) < 0.01) {
                lookaheadPose = new Pose2d(newLookaheadTranslation, turretPose.getRotation());
                lookaheadDist = targetTrans.getDistance(lookaheadPose.getTranslation());
                break;
            }

            lookaheadPose = new Pose2d(newLookaheadTranslation, turretPose.getRotation());
            lookaheadDist = targetTrans.getDistance(lookaheadPose.getTranslation());
        }

        var toGoal = targetTrans.minus(lookaheadPose.getTranslation());
        var turretZeroInFieldFrame = estimatedPose.getRotation().plus(ShooterConstants.TURRET_ZERO_HEADING);

        var turretAngle = toGoal.getAngle().minus(turretZeroInFieldFrame);

        var params = SHOOTER_MAP.get(lookaheadDist);

        return new ShooterSolution(
            Degrees.of(
                MathUtil.clamp(
                    turretAngle.plus(Rotation2d.fromDegrees(LianaHelpers.getTurretAngleAdjustment())).getDegrees(),
                    TurretConstants.MIN_ANGLE.in(Degrees),
                    TurretConstants.MAX_ANGLE.in(Degrees)
                )
            ),
            Degrees.of(params.hood() + LianaHelpers.getHoodAngleAdjustment()),
           RPM.of(params.rpm() + LianaHelpers.getFlywheelAdjustment())
        );
    }

}
