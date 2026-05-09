package frc.robot.subsystems.collectionSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IndexerConstants;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.KickerConstants;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    public void feed() {
        io.set(KickerConstants.FEED, IndexerConstants.FEED);
        Logger.recordOutput("Indexer/KickerTargetOutput", KickerConstants.FEED);
        Logger.recordOutput("Indexer/SpindexerTargetOutput", IndexerConstants.FEED);
    }

    public void clear() {
        io.set(KickerConstants.OUTTAKE, 0.0);
        Logger.recordOutput("Indexer/KickerTargetOutput", KickerConstants.OUTTAKE);
        Logger.recordOutput("Indexer/SpindexerTargetOutput", 0.0);
    }

    public void outtake() {
        io.set(KickerConstants.OUTTAKE, IndexerConstants.OUTTAKE);
        Logger.recordOutput("Indexer/KickerTargetOutput", KickerConstants.OUTTAKE);
        Logger.recordOutput("Indexer/SpindexerTargetOutput", IndexerConstants.OUTTAKE);
    }

    public void stop() {
        io.set(0.0, 0.0);
        Logger.recordOutput("Indexer/KickerTargetOutput", 0.0);
        Logger.recordOutput("Indexer/SpindexerTargetOutput", 0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }
}
