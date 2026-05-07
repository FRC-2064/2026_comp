package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public double positionDeg = 0.0;
        public double appliedVolts = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void setTargetAngle(Angle angle) {}

    default void zeroPosition() {}
}
