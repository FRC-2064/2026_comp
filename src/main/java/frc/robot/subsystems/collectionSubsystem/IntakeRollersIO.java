package frc.robot.subsystems.collectionSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
    @AutoLog
    class IntakeRollersIOInputs {
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
    }

    default void updateInputs(IntakeRollersIOInputs inputs) {}

    default void setOutput(double output) {}
}
