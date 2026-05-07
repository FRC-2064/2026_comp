package frc.robot.subsystems.collectionSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeRollerConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs =
        new IntakeRollersIOInputsAutoLogged();

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
    }

    public void intake() {
        io.setOutput(IntakeRollerConstants.INTAKE);
        Logger.recordOutput("IntakeRollers/TargetOutput", IntakeRollerConstants.INTAKE);
    }

    public void outtake() {
        io.setOutput(IntakeRollerConstants.OUTTAKE);
        Logger.recordOutput("IntakeRollers/TargetOutput", IntakeRollerConstants.OUTTAKE);
    }

    public void stop() {
        io.setOutput(0.0);
        Logger.recordOutput("IntakeRollers/TargetOutput", 0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeRollers", inputs);
    }
}
