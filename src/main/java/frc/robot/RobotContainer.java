// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.TeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredState;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.IntakeExtension;
import frc.robot.subsystems.collectionSubsystem.IntakeRollers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.vision.Vision;
import frc.robot.subsystems.shooterSubsystem.Flywheel;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.utils.ShooterCalc;

public class RobotContainer {

    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);

    // private final Turret turret = new Turret();
    private final Hood hood = new Hood();
    private final Flywheel flywheel = new Flywheel();
    private final IntakeExtension extension = new IntakeExtension();
    private final IntakeRollers rollers = new IntakeRollers();
    private final Indexer indexer = new Indexer();
    private final CommandSwerveDrivetrain drivetrain =
    TunerConstants.createDrivetrain();

    private final ShooterCalc calc = new ShooterCalc(drivetrain);
    private final Vision vision = new Vision(drivetrain);

    private final Superstructure superstructure = new Superstructure(
        extension,
        rollers,
        indexer,
        hood,
        // turret,
        flywheel,
        calc,
        vision,
        () -> operatorXbox.getLeftX()
    );

    private final double MaxSpeed =
        1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake =
        new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final TeleopDrive tdrive = new TeleopDrive(drivetrain, superstructure, driverXbox);

    public RobotContainer() {
        NamedCommands.registerCommand("intake", new RunCommand(() -> superstructure.setDesiredState(DesiredState.INTAKE), superstructure));
        NamedCommands.registerCommand("snowblow", new RunCommand(() -> superstructure.setDesiredState(DesiredState.SNOWBLOW), superstructure));
        NamedCommands.registerCommand("stow", new RunCommand(() -> superstructure.setDesiredState(DesiredState.IDLE), superstructure));
        // NamedCommands.registerCommand("retract intake", new InstantCommand(superstructure::stowIntake));

        configureBindings();
    }

    private void configureBindings() {

        final var leftTrigger = driverXbox.leftTrigger().debounce(0.25);   // intake
        final var rightTrigger = driverXbox.rightTrigger().debounce(0.25); // shoot
        final var leftBumper = driverXbox.leftBumper();     // outtake

        final var back = driverXbox.back();   // toggle drive assist
        final var start = driverXbox.start(); // home hood TESTING ONLY
        final var x = driverXbox.x();         // lock
        final var a = driverXbox.a();         // stow intake
        final var b = driverXbox.b();

        final var oa = operatorXbox.a(); // toggle manual turret manual override
        final var ob = operatorXbox.b(); // manual turret setpoint: 270
        final var ox = operatorXbox.x(); // manual turret setpoint: 90
        final var oy = operatorXbox.y(); // manual turret setpoint: 0

        // TRIGGERS

        leftTrigger
            .and(rightTrigger)
            .whileTrue(
                new RunCommand(() ->
                    superstructure.setDesiredState(DesiredState.SNOWBLOW),
                    superstructure
                )
            );

        leftTrigger
            .and(rightTrigger.negate())
            .whileTrue(
                new RunCommand(() ->
                    superstructure.setDesiredState(DesiredState.INTAKE),
                    superstructure
                )
            );

        rightTrigger
            .and(leftTrigger.negate())
            .whileTrue(
                new RunCommand(() ->
                    superstructure.setDesiredState(DesiredState.SHOOT),
                    superstructure
                )
            );

        leftBumper.whileTrue(
            new RunCommand(() ->
                superstructure.setDesiredState(DesiredState.OUTTAKE),
                superstructure
            )
        );

        // BUTTONS

        //back.onTrue(new InstantCommand( () -> extension.setMotorZero()));
        // a.onTrue(new InstantCommand(extension::extend, extension));
        // b.onTrue(new InstantCommand(extension::stow, extension));
        b.whileTrue(new RunCommand(indexer::feed, indexer));
        x.onTrue(new InstantCommand(indexer::stop));
        // x.onTrue(new InstantCommand(indexer::stop));
        // a.whileTrue(hood.home());
        // x.whileTrue(drivetrain.applyRequest(() -> brake));
        // a.onTrue(new InstantCommand(superstructure::stowIntake));


        // OPERATOR OVERRIDES
        oa.onTrue(new InstantCommand(superstructure::toggleTurretMode));
        oy.onTrue(new InstantCommand(() -> superstructure.setTurretSetpoint(Degrees.of(0))));
        ox.onTrue(new InstantCommand(() -> superstructure.setTurretSetpoint(Degrees.of(90))));
        ob.onTrue(new InstantCommand(() -> superstructure.setTurretSetpoint(Degrees.of(270))));


        // DEFAULT COMMANDS

        superstructure.setDefaultCommand(
            new RunCommand(
                () -> superstructure.setDesiredState(DesiredState.IDLE),
                superstructure
            )
        );

        drivetrain.setDefaultCommand(tdrive);

        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driverXbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("test");
    }
}
