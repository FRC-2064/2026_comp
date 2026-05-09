package frc.robot.subsystems.collectionSubsystem;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtensionIO {
    @AutoLog
    class IntakeExtensionIOInputs {
        public double positionRotations = 0.0;
        public double appliedVolts = 0.0;
    }

    default void updateInputs(IntakeExtensionIOInputs inputs) {}

    default void setTargetPosition(Angle position) {}

    default void zeroPosition() {}
}
