package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class RobotConstants {
    public static TelemetryVerbosity GetTelemetry() {
        if (RobotBase.isSimulation()) {
            return TelemetryVerbosity.HIGH;
        }
        return TelemetryVerbosity.LOW;
    }
}
