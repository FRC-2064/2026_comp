package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.BasicCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.IntakeExtension;
import frc.robot.subsystems.collectionSubsystem.IntakeRollers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.vision.Vision;
import frc.robot.subsystems.shooterSubsystem.Flywheel;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.subsystems.shooterSubsystem.Turret;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.ShooterCalc;

public class RobotContainer {

    // CONTROLLERS

    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);

    // SUBSYSTEMS

    private final Turret turret = new Turret();
    private final Hood hood = new Hood();
    private final Flywheel flywheel = new Flywheel();
    private final IntakeExtension extension = new IntakeExtension();
    private final IntakeRollers rollers = new IntakeRollers();
    private final Indexer indexer = new Indexer();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // UTILITIES

    private final ShooterCalc calc = new ShooterCalc(drivetrain);
    private final Vision vision = new Vision(drivetrain);

    // SUPERSTRUCTURE

    private final Superstructure superstructure = new Superstructure(
        extension,
        rollers,
        indexer,
        hood,
        turret,
        flywheel,
        calc,
        vision,
        () -> operatorXbox.getLeftX()
    );

    private final BasicCommands bcmd = new BasicCommands(superstructure, drivetrain, driverXbox);
    private final Telemetry logger = new Telemetry(RobotConstants.MAX_SPEED);

    public RobotContainer() {
        bcmd.auto.registerAll();
        configureBindings();
    }

    private void configureBindings() {

        // DRIVER BUTTONS

        final var leftTrigger  = driverXbox.leftTrigger().debounce(0.25);  // intake
        final var rightTrigger = driverXbox.rightTrigger().debounce(0.25); // shoot
        final var leftBumper   = driverXbox.leftBumper();                  // outtake
        final var a            = driverXbox.a();                           // stow intake
        final var x            = driverXbox.x();                           // lock drive

        // OPERATOR BUTTONS

        final var oa = operatorXbox.a(); // zero turret encoder
        final var ob = operatorXbox.b(); // turret right (90)
        final var ox = operatorXbox.x(); // turret left (-90)
        final var oy = operatorXbox.y(); // turret center (0)

        // TRIGGERS

        leftTrigger.and(rightTrigger).whileTrue(bcmd.teleop.snowblow);
        leftTrigger.and(rightTrigger.negate()).whileTrue(bcmd.teleop.intake);
        rightTrigger.and(leftTrigger.negate()).whileTrue(bcmd.teleop.shoot);
        leftBumper.whileTrue(bcmd.teleop.outtake);

        // BUTTONS

        a.onTrue(bcmd.teleop.stowIntake);
        x.whileTrue(bcmd.teleop.lockDrive);

        // OPERATOR OVERRIDES

        oa.onTrue(bcmd.teleop.zeroTurret);
        oy.onTrue(bcmd.teleop.turretCenter);
        ox.onTrue(bcmd.teleop.turretLeft);
        ob.onTrue(bcmd.teleop.turretRight);

        // DEFAULT COMMANDS

        superstructure.setDefaultCommand(bcmd.teleop.idle);
        drivetrain.setDefaultCommand(bcmd.teleop.fieldCentricDrive);
        RobotModeTriggers.disabled().whileTrue(bcmd.teleop.disabledIdle);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // AUTO

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("test");
    }
}
