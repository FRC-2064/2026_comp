package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        public double velocityRpm = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double followerAppliedVolts = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setTargetSpeed(AngularVelocity speed) {}

    default void stop() {}
}
