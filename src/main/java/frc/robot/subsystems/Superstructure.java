package frc.robot.subsystems;

import java.util.EnumMap;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.Indexer;
import frc.robot.subsystems.collectionSubsystem.Intake;
import frc.robot.subsystems.collectionSubsystem.Intake.DesiredState;
import frc.robot.subsystems.shooterSubsystem.Hood;
import frc.robot.subsystems.shooterSubsystem.Shooter;
import frc.robot.subsystems.shooterSubsystem.Turret;
import frc.robot.utils.ShooterCalc;

public class Superstructure extends SubsystemBase{
    public enum desiredState{STOW, INTAKE, SHOOT, SNOWBLOW, OUTTAKE}
    
    private desiredState state = desiredState.STOW;

    private final EnumMap<desiredState, Runnable> stateActions;

    private Shooter shooter;
    private Intake intake;
    private Indexer indexer;
    private ShooterCalc shooterCalc;

    public Superstructure(Shooter shooter, Intake intake, Indexer indexer, ShooterCalc shooterCalc){
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.shooterCalc = shooterCalc;

        stateActions = new EnumMap<>(desiredState.class);
        stateActions.put(desiredState.STOW, this::STOW);
        stateActions.put(desiredState.INTAKE, this::INTAKE);
        stateActions.put(desiredState.SHOOT, this::SHOOT);
        stateActions.put(desiredState.SNOWBLOW, this::SNOWBLOW);
        stateActions.put(desiredState.OUTTAKE, this::OUTTAKE);
    }

    private void STOW(){
        intake.setDesiredState(DesiredState.STOWED);
        shooter.stowCommand().schedule();
    }

    private void INTAKE(){
        intake.setDesiredState(DesiredState.INTAKE);
    }

    private void SHOOT(){
        shooter.shootCommand(shooterCalc).schedule();
        indexer.feed();
    }

    private void SNOWBLOW(){
        intake.setDesiredState(DesiredState.INTAKE);
        shooter.shootCommand(shooterCalc).schedule();
        indexer.feed();
    }

    private void OUTTAKE(){
        intake.setDesiredState(DesiredState.OUTTAKE);
        indexer.outtake();
    }

    public void setState(desiredState newState) {
        if (state == newState) {
            return;
        }

        state = newState;
        Runnable action = stateActions.get(newState);
        if (action != null) {
            action.run();
        }
    }

    public desiredState getState() {
        return state;
    }
}