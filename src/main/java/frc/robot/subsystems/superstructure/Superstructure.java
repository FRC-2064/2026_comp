package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.IntakeExtension;
import frc.robot.subsystems.collectionSubsystem.IntakeRollers;
import frc.robot.subsystems.shooterSubsystem.Flywheel;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.subsystems.shooterSubsystem.Turret;
import frc.robot.subsystems.superstructure.SuperstructureEnums.Goal;
import frc.robot.subsystems.superstructure.SuperstructureEnums.Mode;
import frc.robot.subsystems.superstructure.SuperstructureEnums.State;
import frc.robot.utils.Liana.LianaHelpers;
import frc.robot.utils.RobotConstants.SuperstructureConstants;
import frc.robot.utils.ShooterCalc;
import frc.robot.utils.ShooterCalc.ShooterSolution;

public class Superstructure extends SubsystemBase {
    private static final double TELEMETRY_PERIOD_SECONDS = 0.1;
    private static final ShooterSolution ZERO_SOLUTION =
        new ShooterSolution(Degrees.zero(), Degrees.zero(), RPM.zero());

    private final IntakeExtension extension;
    private final IntakeRollers rollers;
    private final Indexer indexer;
    private final Hood hood;
    private final Turret turret;
    private final Flywheel flywheel;
    private final ShooterCalc shooterCalc;

    private Goal goal = Goal.IDLE;
    private State state = State.IDLING;
    private Mode turretMode = Mode.AUTO;
    private Mode shooterMode = Mode.AUTO;

    private ShooterSolution manualSolution = ZERO_SOLUTION;
    private ShooterSolution shooterSolution = ZERO_SOLUTION;
    private boolean shotParametersNeedRefresh = true;
    private boolean readyToShoot = false;
    private double driveSpeedMultiplier = SuperstructureConstants.STOW_SPEED;

    private final Timer kickerClearTimer = new Timer();
    private double lastTelemetryTimestamp = Double.NEGATIVE_INFINITY;

    public Superstructure(
        IntakeExtension extension,
        IntakeRollers rollers,
        Indexer indexer,
        Hood hood,
        Turret turret,
        Flywheel flywheel,
        ShooterCalc shooterCalc
    ) {
        this.extension = extension;
        this.rollers = rollers;
        this.indexer = indexer;
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        this.shooterCalc = shooterCalc;
    }

    @Override
    public void periodic() {
        updateStateMachine();
        updateShooterSolution();
        updateTurret();
        runCurrentState();
        updateTelemetry();
    }

    private void updateStateMachine() {
        State nextState = determineNextState();
        if (nextState != state) {
            transitionTo(nextState);
        }
    }

    private State determineNextState() {
        return switch (goal) {
            case INTAKE -> State.INTAKING;
            case OUTTAKE -> State.OUTTAKING;
            case SHOOT -> readyToShoot ? State.SHOOTING_FEED : State.SHOOTING_SPINUP;
            case SNOWBLOW -> readyToShoot ? State.SNOWBLOW_FEED : State.SNOWBLOW_SPINUP;
            case IDLE -> determineIdleState();
        };
    }

    private State determineIdleState() {
        if (state.isFeeding()) {
            return State.CLEARING_KICKER;
        }

        if (state == State.CLEARING_KICKER
            && !kickerClearTimer.hasElapsed(SuperstructureConstants.KICKER_CLEAR_TIMER)) {
            return State.CLEARING_KICKER;
        }

        return State.IDLING;
    }

    private void transitionTo(State nextState) {
        State previousState = state;
        state = nextState;

        if (state == State.CLEARING_KICKER) {
            kickerClearTimer.restart();
        } else if (previousState == State.CLEARING_KICKER) {
            kickerClearTimer.stop();
            kickerClearTimer.reset();
        }

        if (state.isShooting() && !previousState.isShooting()) {
            shotParametersNeedRefresh = true;
        }
    }

    private void updateShooterSolution() {
        if (!state.isShooting()) {
            shotParametersNeedRefresh = true;
            return;
        }

        ShooterSolution calculatedSolution = shooterCalc.getSelectedSolution();
        var turretAngle = turretMode == Mode.AUTO
            ? calculatedSolution.turretAngle()
            : manualSolution.turretAngle();

        boolean shouldRefreshShotParameters =
            shotParametersNeedRefresh;

        if (shouldRefreshShotParameters) {
            ShooterSolution shotParameters = shooterMode == Mode.AUTO
                ? calculatedSolution
                : manualSolution;

            shooterSolution = new ShooterSolution(
                turretAngle,
                shotParameters.hoodAngle(),
                shotParameters.flywheelVelocity()
            );
            shotParametersNeedRefresh = false;
            return;
        }

        shooterSolution = new ShooterSolution(
            turretAngle,
            shooterSolution.hoodAngle(),
            shooterSolution.flywheelVelocity()
        );
    }

    private void updateTurret() {
        if (state.isShooting()) {
            turret.setTargetAngle(shooterSolution.turretAngle());
        }
    }

    private void runCurrentState() {
        switch (state) {
            case IDLING -> runIdle();
            case INTAKING -> runIntake();
            case OUTTAKING -> runOuttake();
            case SHOOTING_SPINUP -> runShootSpinup();
            case SNOWBLOW_SPINUP -> runSnowblowSpinup();
            case SHOOTING_FEED -> runShootFeed();
            case SNOWBLOW_FEED -> runSnowblowFeed();
            case CLEARING_KICKER -> runClearKicker();
        }
    }

    private void runIdle() {
        driveSpeedMultiplier = SuperstructureConstants.STOW_SPEED;
        readyToShoot = false;

        extension.stow();
        rollers.stop();
        indexer.stop();
        flywheel.stop();
        hood.down();
    }

    private void runIntake() {
        driveSpeedMultiplier = SuperstructureConstants.INTAKE_SPEED;
        readyToShoot = false;

        extension.extend();
        rollers.intake();
        indexer.stop();
        flywheel.stop();
        hood.down();
    }

    private void runOuttake() {
        driveSpeedMultiplier = SuperstructureConstants.OUTTAKE_SPEED;
        readyToShoot = false;

        extension.extend();
        rollers.outtake();
        indexer.outtake();
        flywheel.stop();
        hood.down();
    }

    private void runShootSpinup() {
        driveSpeedMultiplier = SuperstructureConstants.SHOOT_SPEED;

        extension.stow();
        rollers.stop();
        indexer.stop();
        flywheel.setTargetSpeed(shooterSolution.flywheelVelocity());
        hood.setTargetAngle(shooterSolution.hoodAngle());

        readyToShoot = flywheel.isUpToSpeed() && hood.atPosition() && turret.atPosition();
    }

    private void runSnowblowSpinup() {
        driveSpeedMultiplier = SuperstructureConstants.SNOWBLOW_SPEED;

        extension.stow();
        rollers.stop();
        indexer.stop();
        flywheel.setTargetSpeed(shooterSolution.flywheelVelocity());
        hood.setTargetAngle(shooterSolution.hoodAngle());

        readyToShoot = flywheel.isUpToSpeed() && hood.atPosition() && turret.atPosition();
    }

    private void runShootFeed() {
        driveSpeedMultiplier = SuperstructureConstants.SHOOT_SPEED;

        extension.stow();
        rollers.intake();
        indexer.feed();
        flywheel.setTargetSpeed(shooterSolution.flywheelVelocity());
        hood.setTargetAngle(shooterSolution.hoodAngle());
    }

    private void runSnowblowFeed() {
        driveSpeedMultiplier = SuperstructureConstants.SNOWBLOW_SPEED;

        extension.extend();
        rollers.intake();
        indexer.feed();
        flywheel.setTargetSpeed(shooterSolution.flywheelVelocity());
        hood.setTargetAngle(shooterSolution.hoodAngle());
    }

    private void runClearKicker() {
        driveSpeedMultiplier = SuperstructureConstants.STOW_SPEED;
        readyToShoot = false;

        extension.stow();
        rollers.stop();
        indexer.clear();
        flywheel.stop();
        hood.down();
    }

    public boolean isFeeding(State state) {
        return state.isFeeding();
    }

    public boolean isFeeding() {
        return state.isFeeding();
    }

    public boolean isIntaking() {
        return state == State.INTAKING;
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public Goal getGoal() {
        return goal;
    }

    public State getState() {
        return state;
    }

    public Mode getTurretMode() {
        return turretMode;
    }

    public Mode getShooterMode() {
        return shooterMode;
    }

    public double getSpeedMultiplier() {
        return driveSpeedMultiplier;
    }

    public ShooterSolution getCurrentSolution() {
        return shooterSolution;
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    @Deprecated
    public void setDesiredState(Goal goal) {
        setGoal(goal);
    }

    public void toggleTurretMode() {
        turretMode = turretMode == Mode.AUTO ? Mode.MANUAL : Mode.AUTO;
    }

    public void toggleShooterMode() {
        shooterMode = shooterMode == Mode.AUTO ? Mode.MANUAL : Mode.AUTO;
        shotParametersNeedRefresh = true;
    }

    public void setManualSolution(ShooterSolution solution) {
        manualSolution = solution;
        shooterMode = Mode.MANUAL;
        shooterSolution = solution;
        shotParametersNeedRefresh = false;
    }

    @Deprecated
    public void setManuelSol(ShooterSolution solution) {
        setManualSolution(solution);
    }

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

    private void updateTelemetry() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTelemetryTimestamp < TELEMETRY_PERIOD_SECONDS) {
            return;
        }
        lastTelemetryTimestamp = now;

        SmartDashboard.putString("Superstructure/Goal", goal.name());
        SmartDashboard.putString("Superstructure/State", state.name());
        SmartDashboard.putString("Superstructure/TurretMode", turretMode.name());
        SmartDashboard.putString("Superstructure/ShooterMode", shooterMode.name());
        SmartDashboard.putBoolean("Superstructure/ReadyToShoot", readyToShoot);
        SmartDashboard.putBoolean("Superstructure/FlywheelReady", flywheel.isUpToSpeed());
        SmartDashboard.putBoolean("Superstructure/HoodReady", hood.atPosition());
        SmartDashboard.putBoolean("Superstructure/TurretReady", turret.atPosition());
        SmartDashboard.putNumber(
            "Superstructure/SelectedTurretAngle",
            shooterSolution.turretAngle().in(Degrees)
        );
        SmartDashboard.putNumber(
            "Superstructure/SelectedHoodAngle",
            shooterSolution.hoodAngle().in(Degrees)
        );
        SmartDashboard.putNumber(
            "Superstructure/SelectedFlywheelRPM",
            shooterSolution.flywheelVelocity().in(RPM)
        );

        LianaHelpers.updateGameTime();
    }
}
