package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.IntakeExtension;
import frc.robot.subsystems.collectionSubsystem.IntakeRollers;
import frc.robot.subsystems.drive.vision.Vision;
import frc.robot.subsystems.shooterSubsystem.Flywheel;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.subsystems.shooterSubsystem.Turret;
import frc.robot.utils.RobotConstants.SuperstructureConstants;
import frc.robot.utils.ShooterCalc;
import frc.robot.utils.ShooterCalc.ShooterSolution;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {

    // ENUMS

    public enum DesiredState {
        IDLE,
        INTAKE,
        SHOOT,
        SNOWBLOW,
        OUTTAKE,
    }

    public enum State {
        IDLING,
        INTAKING,
        SHOOTING_SPINUP,
        SHOOTING_FEED,
        SNOWBLOW_SPINUP,
        SNOWBLOW_FEED,
        OUTTAKING,
        CLEARING_KICKER,
    }

    // SUBSYSTEMS

    private final IntakeExtension extension;
    private final IntakeRollers rollers;
    private final Indexer indexer;
    private final Hood hood;
    private final Turret turret;
    private final Flywheel flywheel;
    private final Supplier<SwerveDriveState> driveStateSupplier;

    // STATE

    private DesiredState desiredState = DesiredState.IDLE;
    private State currentState = State.IDLING;

    private boolean isManualTargeting = false;
    private ShooterSolution currentSolution = ShooterSolution.zero();

    private boolean isReadyToShoot = false;
    private double speedMult = 1.0;

    // TIMERS

    private final Timer kickerClearTimer = new Timer();

    // CONSTRUCTOR

    public Superstructure(
        Supplier<SwerveDriveState> driveStateSupplier,
        Vision vision
    ) {
        extension = new IntakeExtension();
        rollers = new IntakeRollers();
        indexer = new Indexer();
        hood = new Hood();
        turret = new Turret();
        flywheel = new Flywheel();
        this.driveStateSupplier = driveStateSupplier;
    }

    @Override
    public void periodic() {
        currentState = determineCurrentState();

        if (!isManualTargeting) {
            currentSolution = ShooterCalc.getSelectedSolution(
                driveStateSupplier.get()
            );
        }

        turret.setTargetAngle(currentSolution.turretAngle());
        updateShooter();
    }

    // STATES

    private void updateShooter() {
        switch (currentState) {
            case IDLING:
                idling();
                break;
            case INTAKING:
                intake();
                break;
            case OUTTAKING:
                outtake();
                break;
            case SNOWBLOW_SPINUP:
            case SHOOTING_SPINUP:
                spinup();
                break;
            case SHOOTING_FEED:
                shoot();
                break;
            case SNOWBLOW_FEED:
                snowblow();
                break;
            case CLEARING_KICKER:
                clear();
                break;
        }
    }

    private void idling() {
        speedMult = SuperstructureConstants.STOW_SPEED;
        isReadyToShoot = false;
        rollers.stop();
        indexer.stop();
        flywheel.stop();
        hood.down();
    }

    private void intake() {
        speedMult = SuperstructureConstants.INTAKE_SPEED;
        extension.extend();
        rollers.intake();
        indexer.stop();
        flywheel.stop();
        hood.down();
    }

    private void outtake() {
        speedMult = SuperstructureConstants.OUTTAKE_SPEED;
        extension.extend();
        rollers.outtake();
        indexer.outtake();
        flywheel.stop();
        hood.down();
    }

    private void clear() {
        indexer.clear();
    }

    private void spinup() {
        hood.setTargetAngle(currentSolution.hoodAngle());
        flywheel.setTargetSpeed(currentSolution.flywheelVelocity());
        indexer.stop();

        isReadyToShoot =
            flywheel.isUpToSpeed() && hood.atPosition() && turret.atPosition();
    }

    private void shoot() {
        flywheel.setTargetSpeed(currentSolution.flywheelVelocity());
        hood.setTargetAngle(currentSolution.hoodAngle());
        extension.stow();
        rollers.intake();
        indexer.feed();
    }

    private void snowblow() {
        flywheel.setTargetSpeed(currentSolution.flywheelVelocity());
        hood.setTargetAngle(currentSolution.hoodAngle());
        extension.extend();
        rollers.intake();
        indexer.feed();
    }

    private State determineCurrentState() {
        switch (desiredState) {
            case INTAKE:
                return State.INTAKING;
            case OUTTAKE:
                return State.OUTTAKING;
            case SHOOT:
                return isReadyToShoot
                    ? State.SHOOTING_FEED
                    : State.SHOOTING_SPINUP;
            case SNOWBLOW:
                return isReadyToShoot
                    ? State.SNOWBLOW_FEED
                    : State.SNOWBLOW_SPINUP;
            case IDLE:
            default:
                return resolveIdleState();
        }
    }

    private State resolveIdleState() {
        if (isFeeding(currentState)) {
            kickerClearTimer.restart();
            return State.CLEARING_KICKER;
        }

        if (currentState == State.CLEARING_KICKER) {
            if (
                !kickerClearTimer.hasElapsed(
                    SuperstructureConstants.KICKER_CLEAR_TIMER
                )
            ) {
                return State.CLEARING_KICKER;
            }
            kickerClearTimer.stop();
            kickerClearTimer.reset();
        }

        return State.IDLING;
    }

    // GETTERS

    public boolean isFeeding(State state) {
        return state == State.SHOOTING_FEED || state == State.SNOWBLOW_FEED;
    }

    public boolean isFeeding() {
        return isFeeding(currentState);
    }

    public boolean isIntaking() {
        return currentState == State.INTAKING;
    }

    public boolean isReadyToShoot() {
        return isReadyToShoot;
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

    public void setDesiredState(DesiredState state) {
        this.desiredState = state;
    }

    public void setAutoTargeting() {
        isManualTargeting = false;
    }

    public void setManualSolution(ShooterSolution solution) {
        this.isManualTargeting = true;
        this.currentSolution = solution;
    }

    public void adjustManualTurret(Angle adjustment) {
        if (isManualTargeting) {
            currentSolution = currentSolution.withTurretAngle(
                currentSolution.turretAngle().plus(adjustment)
            );
        }
    }

    // ZEROING
    public void stowIntake() {
        extension.stow();
    }

    public void zeroTurret() {
        turret.zero();
    }

    public void zeroExtension() {
        extension.zero();
    }

    public void zeroHood() {
        hood.zero();
    }
}
