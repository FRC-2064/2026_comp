package frc.robot.commands;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.FieldConstants.LeftTrench;
import frc.robot.utils.RobotConstants.ShooterSolutions;

public class BasicCommands {

    private final Superstructure superstructure;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driver;

    public final AutoCommands auto;
    public final TeleopCommands teleop;

    public BasicCommands(
        Superstructure superstructure,
        CommandSwerveDrivetrain drivetrain,
        CommandXboxController driver
    ) {
        this.superstructure = superstructure;
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.auto = new AutoCommands();
        this.teleop = new TeleopCommands();
    }

    // AUTO COMMANDS

    public class AutoCommands {

        private AutoCommands() {}

        // SUPERSTRUCTURE

        public final Command intake = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.INTAKE),
            superstructure
        );
        public final Command snowblow = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.SNOWBLOW),
            superstructure
        );
        public final Command shoot = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.SHOOT),
            superstructure
        );
        public final Command stow = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.IDLE),
            superstructure
        );

        public final Command setLeftTrench = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.TRENCH_LEFT)
        );

        public final Command setRightTrench = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.TRENCH_RIGHT)
        );

        public final Command setDepot = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.DEPOT)
        );

        public final Command setTower = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.TOWER)
        );

        public void registerAll() {
            NamedCommands.registerCommands(Map.of(
                "intake",   intake,
                "snowblow", snowblow,
                "shoot",    shoot,
                "stow",     stow,
                "leftTrench", setLeftTrench,
                "rightTrench", setRightTrench,
                "depot", setDepot,
                "tower", setTower
            ));
        }
    }

    // TELEOP COMMANDS

    public class TeleopCommands {

        private TeleopCommands() {}

        // SUPERSTRUCTURE

        public final Command intake = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.INTAKE),
            superstructure
        );
        public final Command snowblow = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.SNOWBLOW),
            superstructure
        );
        public final Command shoot = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.SHOOT),
            superstructure
        );
        public final Command outtake = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.OUTTAKE),
            superstructure
        );
        public final Command idle = new RunCommand(
            () -> superstructure.setDesiredState(DesiredState.IDLE),
            superstructure
        );

        // ZEROING

        public final Command stowIntake = new InstantCommand(
            superstructure::stowIntake
        );
        public final Command zeroExtension = new InstantCommand(
            superstructure::zeroExtension
        );
        public final Command zeroHood = new InstantCommand(
            superstructure::zeroHood
        );
        public final Command zeroTurret = new InstantCommand(
            superstructure::zeroTurret
        );

        // MANUEL SHOOTER CONTROL

        public final Command toggleTurretMode = new InstantCommand(
            superstructure::toggleTurretMode
        );
        public final Command toggleShooterMode = new InstantCommand(
            superstructure::toggleShooterMode
        );
        public final Command rigthTrench = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.TRENCH_RIGHT)
        );
        public final Command leftTrench = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.TRENCH_LEFT)
        );
        public final Command tower = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.TOWER)
        );
        public final Command depot = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.DEPOT)
        );
        public final Command humanPlayer = new InstantCommand(
            () -> superstructure.setManuelSol(ShooterSolutions.HUMAN_PLAYER)
        );

        // DRIVE

        private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                .withDeadband(RobotConstants.MAX_SPEED * 0.1)
                .withRotationalDeadband(RobotConstants.MAX_ROT * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
        private final SwerveRequest.SwerveDriveBrake lockRequest = new SwerveRequest.SwerveDriveBrake();

        public final Command fieldCentricDrive = drivetrain.applyRequest(() ->
            driveRequest
                .withVelocityX(-driver.getLeftY() * RobotConstants.MAX_SPEED * superstructure.getSpeedMultiplier())
                .withVelocityY(-driver.getLeftX() * RobotConstants.MAX_SPEED * superstructure.getSpeedMultiplier())
                .withRotationalRate(-driver.getRightX() * RobotConstants.MAX_ROT)
        );
        public final Command disabledIdle = drivetrain.applyRequest(() -> idleRequest)
            .ignoringDisable(true);
        public final Command lockDrive = drivetrain.applyRequest(() -> lockRequest);
    }
}
