package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final SendableChooser<Command> autoChooser;

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
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {

        // DRIVER BUTTONS

        final var lt = driverXbox.leftTrigger().debounce(0.25);    // intake
        final var rt = driverXbox.rightTrigger().debounce(0.25);   // shoot
        final var lb = driverXbox.leftBumper();                    // outtake
        final var a  = driverXbox.a();                             // stow intake
        final var x  = driverXbox.x();                             // lock drive

        // OPERATOR BUTTONS

        final var oa = operatorXbox.a();                           // human player
        final var ob = operatorXbox.b();                           // right trench
        final var ox = operatorXbox.x();                           // left trench
        final var oy = operatorXbox.y();                           // tower
        final var olb = operatorXbox.leftBumper();                 // depot
        final var orb = operatorXbox.rightBumper();                // toggle shooter mode
        final var olt = operatorXbox.leftTrigger().debounce(0.25); // toggle turret mode

        // TRIGGERS

        lt.and(rt).whileTrue(bcmd.teleop.snowblow);
        lt.and(rt.negate()).whileTrue(bcmd.teleop.intake);
        rt.and(lt.negate()).whileTrue(bcmd.teleop.shoot);

        // BUTTONS
        lb.whileTrue(bcmd.teleop.outtake);
        a.onTrue(bcmd.teleop.stowIntake);
        x.whileTrue(bcmd.teleop.lockDrive);

        // OPERATOR OVERRIDES

        ob.onTrue(bcmd.teleop.humanPlayer);
        oa.onTrue(bcmd.teleop.tower);
        olb.onTrue(bcmd.teleop.leftTrench);
        orb.onTrue(bcmd.teleop.rigthTrench);
        ox.onTrue(bcmd.teleop.depot);
        oy.onTrue(bcmd.teleop.toggleTurretMode);

        // DEFAULT COMMANDS

        superstructure.setDefaultCommand(bcmd.teleop.idle);
        drivetrain.setDefaultCommand(bcmd.teleop.fieldCentricDrive);
        RobotModeTriggers.disabled().whileTrue(bcmd.teleop.disabledIdle);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // AUTO

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
