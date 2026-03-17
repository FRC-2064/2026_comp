package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BasicCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.vision.Vision;
import frc.robot.utils.RobotConstants;

public class RobotContainer {

    // CONTROLLERS

    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision vision = new Vision(drivetrain);
    private final SendableChooser<Command> autoChooser;

    // SUPERSTRUCTURE

    private final Superstructure superstructure = new Superstructure(drivetrain::getState, vision);

    private final BasicCommands bcmd = new BasicCommands(superstructure, drivetrain, driverXbox, operatorXbox);
    private final Telemetry logger = new Telemetry(RobotConstants.DriveConstants.MAX_SPEED);

    public RobotContainer() {
        bcmd.auto.registerAll();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {

        // DRIVER BUTTONS

        final var lt = driverXbox.leftTrigger();    // intake
        final var rt = driverXbox.rightTrigger();   // shoot
        final var lb = driverXbox.leftBumper();     // outtake
        final var a  = driverXbox.a();              // stow intake
        final var x  = driverXbox.x();              // lock drive

        // OPERATOR BUTTONS
        final var oa = operatorXbox.a();            // human player
        final var ob = operatorXbox.b();            // right trench
        final var ox = operatorXbox.x();            // left trench
        final var oy = operatorXbox.y();            // tower
        final var olb = operatorXbox.leftBumper();  // depot
        final var orb = operatorXbox.rightBumper(); // Auto Mode

        new Trigger(() -> Math.abs(operatorXbox.getLeftX()) > 0.1)
        .whileTrue(bcmd.teleop.adjustTurret);

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
        orb.onTrue(bcmd.teleop.rightTrench);
        ox.onTrue(bcmd.teleop.depot);
        oy.onTrue(bcmd.teleop.autoMode);

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
