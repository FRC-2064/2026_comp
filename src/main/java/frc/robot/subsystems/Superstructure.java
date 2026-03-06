package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.IntakeExtension;
import frc.robot.subsystems.collectionSubsystem.IntakeRollers;
import frc.robot.subsystems.drive.vision.Vision;
import frc.robot.subsystems.shooterSubsystem.Flywheel;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.subsystems.shooterSubsystem.Turret;
import frc.robot.utils.Liana.LianaHelpers;
import frc.robot.utils.RobotConstants.SuperstructureConstants;
import frc.robot.utils.ShooterCalc;
import frc.robot.utils.ShooterCalc.ShooterSolution;

public class Superstructure extends SubsystemBase {

    // ENUMS

    public enum TurretMode {
        AUTO,
        MANUAL
    }

    public enum DesiredState {
        IDLE,
        INTAKE,
        SHOOT,
        SNOWBLOW,
        OUTTAKE
    }

    public enum State {
        IDLING,
        INTAKING,
        SHOOTING_SPINUP,
        SHOOTING_FEED,
        SNOWBLOW_SPINUP,
        SNOWBLOW_FEED,
        OUTTAKING,
        CLEARING_KICKER
    }

    // SUBSYSTEMS

    private final IntakeExtension extension;
    private final IntakeRollers rollers;
    private final Indexer indexer;
    private final Hood hood;
    private final Turret turret;
    private final Flywheel flywheel;
    private final ShooterCalc shooterCalc;

    // STATE

    private DesiredState desiredState = DesiredState.IDLE;
    private State currentState = State.IDLING;
    private TurretMode turretMode = TurretMode.AUTO;

    private ShooterSolution manualSolution =  new ShooterSolution(Degrees.zero(), Degrees.zero(), RPM.zero());
    private boolean isReadyToShoot = false;
    private double speedMult = 1.0;

    // TURRET

    private Angle manualTurretSetpoint = Degrees.zero();
    private final DoubleSupplier manualTurretAxisSupplier;

    // TIMERS

    private final Debouncer readyToShootDebouncer = new Debouncer(
        SuperstructureConstants.READY_TO_SHOOT_DEBOUNCE_SECONDS
    );
    private final Timer kickerClearTimer = new Timer();

    // CONSTRUCTOR

    public Superstructure(
        IntakeExtension extension,
        IntakeRollers rollers,
        Indexer indexer,
        Hood hood,
        Turret turret,
        Flywheel flywheel,
        ShooterCalc shooterCalc,
        Vision vision,
        DoubleSupplier turretSupplier
    ) {
        this.extension = extension;
        this.rollers = rollers;
        this.indexer = indexer;
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        this.shooterCalc = shooterCalc;
        this.manualTurretAxisSupplier = turretSupplier;
    }

    @Override
    public void periodic() {
        boolean ready = flywheel.isUpToSpeed()
            && hood.atPosition()
            && turret.atPosition();
        isReadyToShoot = readyToShootDebouncer.calculate(ready);

        currentState = determineCurrentState();

        // var solution = shooterCalc.getSelectedSolution();
        // SmartDashboard.putNumber("Turret/SolutionOutput", solution.turretAngle().in(Degrees));

        // if (turretMode == TurretMode.MANUAL){
        //     solution = manualSolution;
        // }


        //var sol = new ShooterSolution(Degrees.zero(), Degrees.of(8), RPM.of(5000)); TURRET SOLUTION
        //var sol = new ShooterSolution(Degrees.zero(), Degrees.of(8), RPM.of(4500)); TOWER SOLUTION
        var sol = new ShooterSolution(Degrees.zero(), Degrees.of(10), RPM.of(5250));

        updateTurret(sol);
        updateShooter(sol);
        updateTelemetry();
    }

    // TURRET CONTROL

    private void updateTurret(ShooterSolution solution) {
        switch (turretMode) {
            case MANUAL:
                double axis = MathUtil.applyDeadband(
                    manualTurretAxisSupplier.getAsDouble(),
                    SuperstructureConstants.MANUAL_TURRET_DEADBAND
                );
                if (Math.abs(axis) > 0) {
                    manualTurretSetpoint = manualTurretSetpoint
                        .plus(SuperstructureConstants.MANUAL_TURRET_RATE.times(axis));
                }
                turret.setTargetAngle(manualTurretSetpoint);
                break;

            case AUTO:
                turret.setTargetAngle(solution.turretAngle());
                manualTurretSetpoint = solution.turretAngle();
                break;
        }
    }

    // STATES

    private void updateShooter(ShooterSolution sol) {
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
            case SHOOTING_SPINUP:
            case SHOOTING_FEED:
                shoot(sol);
                break;
            case SNOWBLOW_SPINUP:
            case SNOWBLOW_FEED:
                snowblow(sol);
                break;
            case CLEARING_KICKER:
                clear();
                break;
        }
    }

    private void idling() {
        speedMult = SuperstructureConstants.STOW_SPEED;
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

    private void shoot(ShooterSolution sol) {
        shooterSequence(
            sol,
            SuperstructureConstants.SHOOT_SPEED,
            false,
            false,
            State.SHOOTING_FEED
        );
    }

    private void snowblow(ShooterSolution sol) {
        shooterSequence(
            sol,
            SuperstructureConstants.SNOWBLOW_SPEED,
            true,
            true,
            State.SNOWBLOW_FEED
        );
    }

    private void shooterSequence(
        ShooterSolution sol,
        double speedMult,
        boolean extendIntake,
        boolean runRollers,
        State feedState
    ) {
        this.speedMult = speedMult;

        if (extendIntake) {
            extension.extend();
        } else {
            if ((Timer.getFPGATimestamp() % 0.6) > 0.3) {
                extension.extend();
            } else {
                extension.stow();
            }
        }

        if (runRollers) {
            rollers.intake();
        } else {
            rollers.stop();
        }

        flywheel.setTargetSpeed(sol.flywheelVelocity());
        hood.setTargetAngle(sol.hoodAngle());

        if (currentState == feedState) {
            indexer.feed();
        } else if (currentState != State.CLEARING_KICKER) {
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
                return resolveShooterState(State.SHOOTING_FEED, State.SHOOTING_SPINUP);
            case SNOWBLOW:
                return resolveShooterState(State.SNOWBLOW_FEED, State.SNOWBLOW_SPINUP);
            case IDLE:
            default:
                return resolveIdleState();
        }
    }

    private State resolveShooterState(State feedState, State spinupState) {
        if (currentState == feedState && !isReadyToShoot()) {
            kickerClearTimer.restart();
            return State.CLEARING_KICKER;
        }

        if (currentState == State.CLEARING_KICKER) {
            if (!kickerClearTimer.hasElapsed(SuperstructureConstants.KICKER_CLEAR_TIMER)) {
                return State.CLEARING_KICKER;
            }
            kickerClearTimer.stop();
            kickerClearTimer.reset();
            return spinupState;
        }

        return isReadyToShoot() ? feedState : spinupState;
    }

    private State resolveIdleState() {
        if (isFeeding(currentState)) {
            kickerClearTimer.restart();
            return State.CLEARING_KICKER;
        }

        if (currentState == State.CLEARING_KICKER) {
            if (!kickerClearTimer.hasElapsed(SuperstructureConstants.KICKER_CLEAR_TIMER)) {
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

    public TurretMode getTurretMode() {
        return turretMode;
    }

    public double getSpeedMultiplier() {
        return speedMult;
    }

    public ShooterSolution getCurrentSolution() {
        return shooterCalc.getSelectedSolution();
    }

    // SETTERS

    public void setDesiredState(DesiredState state) {
        this.desiredState = state;
    }

    public void setTurretSetpoint(Angle setpoint) {
        this.manualTurretSetpoint = setpoint;
        this.turretMode = TurretMode.MANUAL;
    }

    public void toggleTurretMode() {
        turretMode = turretMode == TurretMode.AUTO ? TurretMode.MANUAL : TurretMode.AUTO;
    }

    public void setManuelSol(ShooterSolution manuelSol){
        manualSolution = manuelSol;
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

    // TELEMETRY

    private void updateTelemetry() {
        SmartDashboard.putString("Superstructure/DesiredState", desiredState.name());
        SmartDashboard.putString("Superstructure/CurrentState", currentState.name());
        SmartDashboard.putBoolean("Superstructure/ReadyToShoot", isReadyToShoot());
        SmartDashboard.putBoolean("Superstructure/FlywheelReady", flywheel.isUpToSpeed());
        SmartDashboard.putBoolean("Superstructure/HoodReady", hood.atPosition());
        SmartDashboard.putBoolean("Superstructure/TurretReady", turret.atPosition());
        SmartDashboard.putString("Superstructure/TurretMode", turretMode.name());
        SmartDashboard.putNumber("Superstructure/TurretManualAngle", manualTurretSetpoint.in(Degrees));

        LianaHelpers.updateGameTime();
    }
}
