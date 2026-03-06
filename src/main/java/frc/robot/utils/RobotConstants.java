package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class RobotConstants {

    public static TelemetryVerbosity GetTelemetry() {
        if (RobotBase.isSimulation()) {
            return TelemetryVerbosity.HIGH;
        }
        return TelemetryVerbosity.LOW;
    }

    public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ROT = RotationsPerSecond.of(0.75).in(RadiansPerSecond);


    public static class SuperstructureConstants {

        public static final double READY_TO_SHOOT_DEBOUNCE_SECONDS = 0.5;

        public static final Time KICKER_CLEAR_TIMER = Seconds.of(0.25);

        public static final double STOW_SPEED = 1.0;
        public static final double OUTTAKE_SPEED = 1.0;
        public static final double INTAKE_SPEED = 0.75;
        public static final double SHOOT_SPEED = 0.75;
        public static final double SNOWBLOW_SPEED = 0.25;

        public static final double MANUAL_TURRET_DEADBAND = 0.1;
        public static final Angle MANUAL_TURRET_RATE = Degrees.of(3);
    }

    public static final CANBus CANIVORE = new CANBus("Comp");
}
