package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.IntakeExtension;
import frc.robot.subsystems.collectionSubsystem.IntakeRollers;
import frc.robot.subsystems.drive.vision.Vision;
import frc.robot.subsystems.shooterSubsystem.FlywheelSubsystem;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooterSubsystem.Turret;
import frc.robot.utils.ShooterCalc;
import frc.robot.utils.ShooterCalc.ShooterSolution;

public class Superstructure extends SubsystemBase {

    public enum DesiredState {
        STOW,
        INTAKE,
        SHOOT,
        SNOWBLOW,
        OUTTAKE
    }

    public enum State {
        STOWED,
        INTAKING,
        SHOOTING_SPINUP,
        SHOOTING_FEED,
        SNOWBLOW_SPINUP,
        SNOWBLOW_FEED,
        OUTTAKING
    }

    private DesiredState desiredState = DesiredState.STOW;
    private State currentState = State.STOWED;

    private final IntakeExtension extension;
    private final IntakeRollers rollers;
    private final Indexer indexer;

    private final Hood hood;
    private final Turret turret;
    private final FlywheelSubsystem flywheel;

    private final ShooterCalc shooterCalc;
    private final Vision vision;

    private final Debouncer readyToShootDebouncer = new Debouncer(0.25);

    private double speedMult = 1.0;

    public Superstructure(
        IntakeExtension extension,
        IntakeRollers rollers,
        Indexer indexer,
        Hood hood,
        Turret turret,
        FlywheelSubsystem flywheel,
        ShooterCalc shooterCalc,
        Vision vision
    ) {
        this.extension = extension;
        this.rollers = rollers;
        this.indexer = indexer;
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        this.shooterCalc = shooterCalc;
        this.vision = vision;
    }

    public void setDesiredState(DesiredState state) {
        this.desiredState = state;
    }

    public void stowIntake() {
        extension.stow();
    }

    public DesiredState getDesiredState() {
        return desiredState;
    }

    public State getState() {
        return currentState;
    }

    public double getSpeedMultiplier() {
        return speedMult;
    }

    @Override
    public void periodic() {
        currentState = determineCurrentState();

        var solution = shooterCalc.getSelectedSolution();
        turret.setTargetAngle(solution.turretAngle());

        switch (currentState) {
            case STOWED:
                stow();
                break;
            case INTAKING:
                intake();
                break;
            case OUTTAKING:
                outtake();
                break;
            case SHOOTING_SPINUP:
            case SHOOTING_FEED:
                shoot(solution);
                break;
            case SNOWBLOW_SPINUP:
            case SNOWBLOW_FEED:
                snowblow(solution);
                break;
        }

        updateTelemetry();
    }

    public boolean isFeeding() {
        return (
            currentState == State.SHOOTING_FEED ||
            currentState == State.SNOWBLOW_FEED
        );
    }

    public boolean isIntaking() {
        return currentState == State.INTAKING;
    }

    public ShooterSolution getCurrentSolution() {
        return shooterCalc.getSelectedSolution();
    }

    private void stow() {
        speedMult = 1.0;
        rollers.stop();
        indexer.stop();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
        hood.setTargetAngle(HoodConstants.STARTING_POS);
    }

    private void intake() {
        speedMult = 0.75;
        extension.extend();
        rollers.intake();
        indexer.stop();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
        hood.setTargetAngle(HoodConstants.STARTING_POS);
    }

    private void outtake() {
        speedMult = 1.0;
        extension.extend();
        rollers.outtake();
        indexer.outtake();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
    }

    private void shoot(ShooterSolution sol) {
        speedMult = 0.75;

        extension.stow();
        rollers.intake();

        flywheel.setTargetSpeed(sol.flywheelVelocity());
        hood.setTargetAngle(sol.hoodAngle());

        if (currentState == State.SHOOTING_FEED) {
            indexer.feed();
        } else {
            indexer.stop();
        }
    }

    private void snowblow(ShooterSolution sol) {
        speedMult = 0.25;

        extension.extend();
        rollers.intake();

        flywheel.setTargetSpeed(sol.flywheelVelocity());
        hood.setTargetAngle(sol.hoodAngle());

        if (currentState == State.SNOWBLOW_FEED) {
            indexer.feed();
        } else {
            indexer.stop();
        }
    }

    private State determineCurrentState() {
        switch (desiredState) {
            case INTAKE:
                return State.INTAKING;
            case OUTTAKE:
                return State.OUTTAKING;
            case SHOOT:
                return isReadyToShoot()
                    ? State.SHOOTING_FEED
                    : State.SHOOTING_SPINUP;
            case SNOWBLOW:
                return isReadyToShoot()
                    ? State.SNOWBLOW_FEED
                    : State.SNOWBLOW_SPINUP;

            case STOW:
            default:
                return State.STOWED;
        }
    }

    public boolean isReadyToShoot() {
        boolean ready = flywheel.isUpToSpeed() && hood.atPosition() && turret.atPosition();
        return readyToShootDebouncer.calculate(ready);
    }

    private void updateTelemetry() {
        SmartDashboard.putString(
            "Superstructure/DesiredState",
            desiredState.name()
        );
        SmartDashboard.putString(
            "Superstructure/CurrentState",
            currentState.name()
        );
        SmartDashboard.putBoolean(
            "Superstructure/ReadyToShoot",
            isReadyToShoot()
        );
        SmartDashboard.putBoolean(
            "Superstructure/FlywheelReady",
            flywheel.isUpToSpeed()
        );
        SmartDashboard.putBoolean(
            "Superstructure/HoodReady",
            hood.atPosition()
        );
        SmartDashboard.putBoolean(
            "Superstructure/TurretReady",
            turret.atPosition()
        );
    }
}
