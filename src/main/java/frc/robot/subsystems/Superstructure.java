package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.Intake;
import frc.robot.subsystems.collectionSubsystem.Intake.IntakeState;
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
        OUTTAKE,
        DEPLOYED
    }

    public enum State {
        STOWED,
        INTAKING,
        SHOOTING_SPINUP,
        SHOOTING_FEED,
        SNOWBLOW_SPINUP,
        SNOWBLOW_FEED,
        OUTTAKING,
        DEPLOYED
    }

    private DesiredState desiredState = DesiredState.STOW;
    private State currentState = State.STOWED;

    private final Intake intake;
    private final Indexer indexer;

    private final Hood hood;
    private final Turret turret;
    private final FlywheelSubsystem flywheel;

    private final ShooterCalc shooterCalc;

    private Time readyToShootTimer = Milliseconds.of(0);
    private boolean wasReadyLastCycle = false;

    private double speedMult = 1.0;

    public Superstructure(
        Intake intake,
        Indexer indexer,
        Hood hood,
        Turret turret,
        FlywheelSubsystem flywheel,
        ShooterCalc shooterCalc
    ) {
        this.intake = intake;
        this.indexer = indexer;
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        this.shooterCalc = shooterCalc;
    }

    public void setDesiredState(DesiredState state) {
        this.desiredState = state;
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
            case DEPLOYED:
                deployed();
        }

        updateTelemetry();
    }

    public boolean isReadyToFire() {
        return isReadyToShoot();
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
        intake.setDesiredState(IntakeState.STOWED);
        indexer.stop();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
        hood.setTargetAngle(HoodConstants.STARTING_POS);
    }

    private void intake() {
        speedMult = 0.75;
        intake.setDesiredState(IntakeState.INTAKE);
        indexer.stop();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
        hood.setTargetAngle(HoodConstants.STARTING_POS);
    }

    private void deployed(){
        indexer.stop();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
        hood.setTargetAngle(HoodConstants.STARTING_POS);
        intake.setDesiredState(IntakeState.DEPLOYED);
    }

    private void outtake() {
        speedMult = 1.0;
        intake.setDesiredState(IntakeState.OUTTAKE);
        indexer.outtake();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
    }

    private void shoot(ShooterSolution sol) {
        speedMult = 0.75;
        intake.setDesiredState(IntakeState.STOWED);

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
        intake.setDesiredState(IntakeState.INTAKE);

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
            case DEPLOYED:
                return State.DEPLOYED;
            default:
                return State.STOWED;
        }
    }

    private boolean isReadyToShoot() {
        var isReady =
            flywheel.isUpToSpeed() && hood.atPosition() && turret.atPosition();
        if (isReady && !wasReadyLastCycle) {
            readyToShootTimer = Milliseconds.of(0);
        } else if (isReady && !wasReadyLastCycle) {
            readyToShootTimer.plus(Milliseconds.of(20));
        } else {
            readyToShootTimer = Milliseconds.zero();
        }

        wasReadyLastCycle = isReady;
        return readyToShootTimer.gte(
            SuperstructureConstants.READY_TO_SHOOT_DEBOUNCE_TIME
        );
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
        SmartDashboard.putNumber(
            "Superstructure/ReadyTimer",
            readyToShootTimer.in(Seconds)
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
