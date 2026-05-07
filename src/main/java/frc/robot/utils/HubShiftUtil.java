package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.Supplier;

public class HubShiftUtil {
  private static final double TELEMETRY_PERIOD_SECONDS = 0.1;
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static double lastTelemetryTimestamp = Double.NEGATIVE_INFINITY;
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

  private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  public static void setAllianceWinOverride(Supplier<Optional<Boolean>> override) {
    allianceWinOverride = override;
  }

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Return override value
    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Return default value
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the beginning of teleop. */
  public static void initialize() {
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double currentTime = shiftTimer.get();
    double stateTimeElapsed = currentTime;
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }

    return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  /**
   * Periodically updates SmartDashboard with the current hub shift status.
   * Call this in your Robot.java robotPeriodic() or a subsystem periodic() loop.
   */
  public static void updateDashboardTelemetry() {
    double now = Timer.getFPGATimestamp();
    if (now - lastTelemetryTimestamp < TELEMETRY_PERIOD_SECONDS) {
      return;
    }
    lastTelemetryTimestamp = now;

    ShiftInfo currentShift = getOfficialShiftInfo();

    boolean isOurAllianceActive = currentShift.active();
    double timeRemaining = currentShift.remainingTime();
    boolean lessThanThreeSeconds = timeRemaining < 3.0 && timeRemaining > 0.0;

    SmartDashboard.putBoolean("Hub/Is Alliance Active", isOurAllianceActive);
    SmartDashboard.putNumber("Hub/Shift Time Remaining", timeRemaining);
    SmartDashboard.putBoolean("Hub/Shift Ending Soon (<3s)", lessThanThreeSeconds);
  }
}
