package frc.robot.subsystems.collectionSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public double spindexerAppliedVolts = 0.0;
        public double kickerAppliedVolts = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void set(double kickerOutput, double spindexerOutput) {}
}
