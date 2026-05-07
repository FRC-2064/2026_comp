package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public double positionDeg = 0.0;
        public double appliedVolts = 0.0;
        public double absoluteSmallRotations = 0.0;
        public double absoluteLargeRotations = 0.0;
        public String absoluteSolveStatus = "NO_MATCH";
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setTargetAngle(Angle angle) {}

    default void zeroPosition() {}
}
