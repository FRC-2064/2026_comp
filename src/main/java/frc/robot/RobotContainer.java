// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredState;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.Intake;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.vision.Vision;
import frc.robot.subsystems.shooterSubsystem.FlywheelSubsystem;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.subsystems.shooterSubsystem.Turret;
import frc.robot.utils.ShooterCalc;

public class RobotContainer {

    final CommandXboxController driverXbox = new CommandXboxController(0);

    private final Turret turret = new Turret();
    private final Hood hood = new Hood();
    private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
    private final Intake intake = new Intake();
    private final Indexer indexer = new Indexer();

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();
    private final ShooterCalc calc = new ShooterCalc(drivetrain);
    private final Vision vision = new Vision(drivetrain);
    private final Superstructure superstructure = new Superstructure(
        intake,
        indexer,
        hood,
        turret,
        flywheel,
        calc
    );

    private final double MaxSpeed =
        1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(
        RadiansPerSecond
    );

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake =
        new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point =
        new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public RobotContainer() {
        NamedCommands.registerCommand("intake", new RunCommand(() -> superstructure.setDesiredState(DesiredState.INTAKE)));
        NamedCommands.registerCommand("shoot", new RunCommand(() -> superstructure.setDesiredState(DesiredState.SHOOT)));
        NamedCommands.registerCommand("snowblow", new RunCommand(() - superstructure.setDesiredState(DesiredState.SNOWBLOW)));
        NamedCommands.registerCommand("stow", new RunCommand(() -> superstructure.setDesiredState(DesiredState.STOW)));
    
        configureBindings();
    }

    private void configureBindings() {
        final var leftTrigger = driverXbox.leftTrigger();
        final var rightTrigger = driverXbox.rightTrigger();
        final var leftBumper = driverXbox.leftBumper();

        leftTrigger
            .and(rightTrigger)
            .whileTrue(
                new RunCommand(() ->
                    superstructure.setDesiredState(DesiredState.SNOWBLOW)
                )
            );

        leftTrigger
            .and(rightTrigger.negate())
            .whileTrue(
                new RunCommand(() ->
                    superstructure.setDesiredState(DesiredState.INTAKE)
                )
            );

        rightTrigger
            .and(leftTrigger.negate())
            .whileTrue(
                new RunCommand(() ->
                    superstructure.setDesiredState(DesiredState.SHOOT)
                )
            );

        leftBumper.whileTrue(
            new RunCommand(() ->
                superstructure.setDesiredState(DesiredState.OUTTAKE)
            )
        );

        superstructure.setDefaultCommand(
            new RunCommand(
                () -> superstructure.setDesiredState(DesiredState.STOW),
                superstructure
            )
        );

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                final double speedMult = superstructure.getSpeedMultiplier();
                return drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * speedMult) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedMult) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX() * MaxAngularRate * speedMult
                    ); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick
            .b()
            .whileTrue(
                drivetrain.applyRequest(() ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -joystick.getLeftY(),
                            -joystick.getLeftX()
                        )
                    )
                )
            );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick
            .back()
            .and(joystick.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick
            .back()
            .and(joystick.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick
            .start()
            .and(joystick.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick
            .start()
            .and(joystick.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick
            .leftBumper()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() ->
                drivetrain.seedFieldCentric(Rotation2d.kZero)
            ),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain
                .applyRequest(() ->
                    drive
                        .withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                )
                .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
