package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.Intake;
import frc.robot.subsystems.collectionSubsystem.Intake.IntakeState;
import frc.robot.subsystems.shooterSubsystem.FlywheelSubsystem;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import frc.robot.subsystems.shooterSubsystem.Turret;
import frc.robot.utils.ShooterCalc;
import frc.robot.utils.ShooterCalc.ShooterSolution;

public class Superstructure extends SubsystemBase{
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

    private final Intake intake;
    private final Indexer indexer;

    private final Hood hood;
    private final Turret turret;
    private final FlywheelSubsystem flywheel;

    private final ShooterCalc shooterCalc;

    public Superstructure(Intake intake, Indexer indexer, Hood hood, Turret turret, FlywheelSubsystem flywheel,
            ShooterCalc shooterCalc) {
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
        if (desiredState == DesiredState.SNOWBLOW) {
            return 0.1;
        }
        return 1.0;
    }

    @Override
    public void periodic() {
        switch (desiredState) {
            case INTAKE:
                currentState = State.INTAKING;
                break;
            case OUTTAKE:
                currentState = State.OUTTAKING;
                break;
            case SHOOT:
            currentState = (isReadyToShoot()) ? State.SHOOTING_FEED : State.SHOOTING_SPINUP;
                break;
            case SNOWBLOW:
            currentState = (isReadyToShoot()) ? State.SNOWBLOW_FEED : State.SNOWBLOW_SPINUP;
                break;
            default:
                currentState = State.STOWED;
                break;
        }

        var solution = shooterCalc.getSelectedSolution();
        turret.setTargetAngle(solution.turretAngle());

        switch (currentState) {
            case STOWED: stow(); break;
            case INTAKING: intake(); break;
            case OUTTAKING: outtake(); break;
            case SHOOTING_SPINUP:
            case SHOOTING_FEED: shoot(solution); break;
            case SNOWBLOW_SPINUP:
            case SNOWBLOW_FEED: snowblow(solution); break;
        }
    }

    private void stow() {
        intake.setDesiredState(IntakeState.STOWED);
        indexer.stop();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
        hood.setTargetAngle(HoodConstants.STARTING_POS);
    }

    private void intake() {
        intake.setDesiredState(IntakeState.INTAKE);
        indexer.stop();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
        hood.setTargetAngle(HoodConstants.STARTING_POS);
    }

    private void outtake() {
        intake.setDesiredState(IntakeState.OUTTAKE);
        indexer.outtake();
        flywheel.setTargetSpeed(FlyWheelConstants.MIN_VELOCITY);
    }

private void shoot(ShooterSolution sol) {
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
    intake.setDesiredState(IntakeState.INTAKE);
    
    flywheel.setTargetSpeed(sol.flywheelVelocity());
    hood.setTargetAngle(sol.hoodAngle());

    if (currentState == State.SNOWBLOW_FEED) {
        indexer.feed();
    } else {
        indexer.stop();
    }
}

    private boolean isReadyToShoot() {
        return flywheel.isUpToSpeed()
                && hood.atPosition()
                && turret.atPosition();
    }
}