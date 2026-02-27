package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.RobotConstants.DriveConstants;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {

    private final CommandSwerveDrivetrain drive;
    private final Superstructure superstructure;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private int flipFactor = 1;

    private LinearVelocity currentMaxDriveSpeed =
        DriveConstants.MAX_DRIVE_SPEED;
    private AngularVelocity currentMaxRotSpeed = DriveConstants.MAX_ROT_SPEED;

    private final Trigger inTrenchZone = new Trigger(
        this::inTrenchZone
    ).debounce(0.1);
    private final Trigger inBumpZone = new Trigger(this::inBumpZone).debounce(
        0.1
    );

    private final PIDController trenchYController = new PIDController(
        4.0,
        0,
        0
    );
    private final PIDController rotationController = new PIDController(
        5.0,
        0,
        0.1
    );

    private static final double ROBOT_HALF_WIDTH = 0.4255;
    private static final double ROTATION_EARLY_TRIGGER = 0.01;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(
            DriveRequestType.OpenLoopVoltage
        );

    private DriveMode currentDriveMode = DriveMode.NORMAL;

    private boolean zoneAssistEnabled = true;

    public enum DriveMode {
        NORMAL,
        TRENCH_LOCK,
        BUMP_LOCK,
    }

    public TeleopDrive(
        CommandSwerveDrivetrain drive,
        Superstructure superstructure,
        CommandXboxController controller
    ) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.xSupplier = () -> -controller.getLeftY() * flipFactor;
        this.ySupplier = () -> -controller.getLeftX() * flipFactor;
        this.rotSupplier = () -> -controller.getRightX();

        this.xLimiter = new SlewRateLimiter(
            DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)
        );
        this.yLimiter = new SlewRateLimiter(
            DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)
        );
        inTrenchZone.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
        inBumpZone.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
        inTrenchZone.or(inBumpZone).onFalse(updateDriveMode(DriveMode.NORMAL));

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(2.0));
        trenchYController.setTolerance(0.05);

        addRequirements(drive);
    }

    private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = MathUtil.applyDeadband(
            Math.hypot(x, y),
            DriveConstants.DEADBAND
        );
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        linearMagnitude = linearMagnitude * linearMagnitude;

        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(
                new Transform2d(linearMagnitude, 0.0, new Rotation2d())
            )
            .getTranslation();
    }

    private boolean poseInZone(
        Pose2d robotPose,
        double minX,
        double maxX,
        double minY,
        double maxY,
        double xExpansion,
        double yExpansion
    ) {
        return (
            robotPose.getX() + ROBOT_HALF_WIDTH >= minX - xExpansion &&
            robotPose.getX() - ROBOT_HALF_WIDTH <= maxX + xExpansion &&
            robotPose.getY() + ROBOT_HALF_WIDTH >= minY - yExpansion &&
            robotPose.getY() - ROBOT_HALF_WIDTH <= maxY + yExpansion
        );
    }

    private boolean inTrenchZone() {
        if (!zoneAssistEnabled) return false;
        Pose2d robotPose = drive.getState().Pose;

        double trenchMinX =
            FieldConstants.LinesVertical.hubCenter - Units.inchesToMeters(60);
        double trenchMaxX =
            FieldConstants.LinesVertical.hubCenter + Units.inchesToMeters(60);

        // Opposing side X range
        double oppTrenchMinX =
            FieldConstants.LinesVertical.oppHubCenter -
            Units.inchesToMeters(60);
        double oppTrenchMaxX =
            FieldConstants.LinesVertical.oppHubCenter +
            Units.inchesToMeters(60);

        boolean inLeftTrench = poseInZone(
            robotPose,
            trenchMinX,
            trenchMaxX,
            FieldConstants.fieldWidth - FieldConstants.LeftTrench.openingWidth,
            FieldConstants.fieldWidth,
            ROTATION_EARLY_TRIGGER,
            0.0
        );

        boolean inRightTrench = poseInZone(
            robotPose,
            trenchMinX,
            trenchMaxX,
            0,
            FieldConstants.RightTrench.openingWidth,
            ROTATION_EARLY_TRIGGER,
            0.0
        );

        boolean inOppLeftTrench = poseInZone(
            robotPose,
            oppTrenchMinX,
            oppTrenchMaxX,
            FieldConstants.fieldWidth - FieldConstants.LeftTrench.openingWidth,
            FieldConstants.fieldWidth,
            ROTATION_EARLY_TRIGGER,
            0.0
        );

        boolean inOppRightTrench = poseInZone(
            robotPose,
            oppTrenchMinX,
            oppTrenchMaxX,
            0,
            FieldConstants.RightTrench.openingWidth,
            ROTATION_EARLY_TRIGGER,
            0.0
        );

        return (
            inLeftTrench || inRightTrench || inOppLeftTrench || inOppRightTrench
        );
    }

    private boolean inBumpZone() {
        if (!zoneAssistEnabled) return false;
        if (inTrenchZone()) return false;
        Pose2d robotPose = drive.getState().Pose;

        double bumpMinX =
            FieldConstants.LinesVertical.hubCenter -
            FieldConstants.LeftBump.width / 2;
        double bumpMaxX =
            FieldConstants.LinesVertical.hubCenter +
            FieldConstants.LeftBump.width / 2;

        double oppBumpMinX =
            FieldConstants.LinesVertical.oppHubCenter -
            FieldConstants.LeftBump.width / 2;
        double oppBumpMaxX =
            FieldConstants.LinesVertical.oppHubCenter +
            FieldConstants.LeftBump.width / 2;

        boolean inLeftBump = poseInZone(
            robotPose,
            bumpMinX,
            bumpMaxX,
            FieldConstants.LinesHorizontal.leftBumpEnd,
            FieldConstants.LinesHorizontal.leftBumpStart,
            0.0,
            0.0
        );

        boolean inRightBump = poseInZone(
            robotPose,
            bumpMinX,
            bumpMaxX,
            FieldConstants.LinesHorizontal.rightBumpEnd,
            FieldConstants.LinesHorizontal.rightBumpStart,
            0.0,
            0.0
        );

        boolean inOppLeftBump = poseInZone(
            robotPose,
            oppBumpMinX,
            oppBumpMaxX,
            FieldConstants.LinesHorizontal.leftBumpEnd,
            FieldConstants.LinesHorizontal.leftBumpStart,
            0.0,
            0.0
        );

        boolean inOppRightBump = poseInZone(
            robotPose,
            oppBumpMinX,
            oppBumpMaxX,
            FieldConstants.LinesHorizontal.rightBumpEnd,
            FieldConstants.LinesHorizontal.rightBumpStart,
            0.0,
            0.0
        );

        return inLeftBump || inRightBump || inOppLeftBump || inOppRightBump;
    }

    private Rotation2d getTrenchLockAngle() {
        return Rotation2d.kZero;
    }

    private double getTrenchY() {
        Pose2d robotPose = drive.getState().Pose;
        double trenchCenterY = FieldConstants.LeftTrench.openingWidth / 2.0;

        if (robotPose.getY() >= FieldConstants.fieldWidth / 2.0) {
            return FieldConstants.fieldWidth - trenchCenterY;
        }

        return trenchCenterY;
    }

    private Rotation2d getBumpLockAngle() {
        for (int i = -135; i < 180; i += 90) {
            if (
                Math.abs(
                    MathUtil.inputModulus(
                        drive.getState().Pose.getRotation().getDegrees() - i,
                        -180,
                        180
                    )
                ) <=
                45
            ) {
                return Rotation2d.fromDegrees(i);
            }
        }
        return Rotation2d.fromDegrees(0);
    }

    private Command updateDriveMode(DriveMode mode) {
        return Commands.runOnce(() -> currentDriveMode = mode);
    }

    public Command toggleZoneAssist() {
        return Commands.runOnce(() -> zoneAssistEnabled = !zoneAssistEnabled);
    }

    @Override
    public void initialize() {
        flipFactor =
            DriverStation.getAlliance().isPresent() &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ? -1
                : 1;
    }

    @Override
    public void execute() {
        Translation2d linearVel = getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(),
            ySupplier.getAsDouble()
        );
        double limitedX = xLimiter.calculate(linearVel.getX());
        double limitedY = yLimiter.calculate(linearVel.getY());

        double speedMult = superstructure.getSpeedMultiplier();

        double maxSpeedMPS = currentMaxDriveSpeed.in(MetersPerSecond);
        double targetX = limitedX * maxSpeedMPS * speedMult;
        double targetY = limitedY * maxSpeedMPS * speedMult;

        double rot = 0.0;

        switch (currentDriveMode) {
            case NORMAL:
                double rawRot = MathUtil.applyDeadband(
                    rotSupplier.getAsDouble(),
                    DriveConstants.DEADBAND
                );
                rawRot = Math.copySign(rawRot * rawRot, rawRot);

                double maxrotRPS = currentMaxRotSpeed.in(RadiansPerSecond);
                rot = rawRot * maxrotRPS * speedMult;
                break;
            case TRENCH_LOCK:
                trenchYController.setSetpoint(getTrenchY());
                targetY = trenchYController.calculate(
                    drive.getState().Pose.getY()
                );
                if (trenchYController.atSetpoint()) targetY = 0;

                rotationController.setSetpoint(
                    getTrenchLockAngle().getRadians()
                );
                rot = rotationController.calculate(
                    drive.getState().Pose.getRotation().getRadians()
                );
                if (rotationController.atSetpoint()) rot = 0;
                break;
            case BUMP_LOCK:
                rotationController.setSetpoint(getBumpLockAngle().getRadians());
                rot = rotationController.calculate(
                    drive.getState().Pose.getRotation().getRadians()
                );
                if (rotationController.atSetpoint()) rot = 0;
                break;
        }

        drive.setControl(
            driveRequest
                .withVelocityX(targetX)
                .withVelocityY(targetY)
                .withRotationalRate(rot)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
