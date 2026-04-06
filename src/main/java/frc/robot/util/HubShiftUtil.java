// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Setter;

public class HubShiftUtil {
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
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  // ===== SIMPLIFIED TIMING OFFSETS =====
  // Tune these at comp
  private static final double shootEarlyOffset = -2.0; // start shooting early
  private static final double shootLateOffset = 0.0;  // stop shooting
  
  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;

  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

  private static final double timeResetThreshold = 3.0;
  private static double shiftTimerOffset = 0.0;

  @Setter private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Override
    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Default fallback
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the beginning of teleop. */
  public static void initialize() {
    shiftTimerOffset = 0;
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    Alliance startAlliance = getFirstActiveAlliance();
    return startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
        ? activeSchedule
        : inactiveSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {

    double timerValue = shiftTimer.get();
    double currentTime = timerValue - shiftTimerOffset;

    double stateTimeElapsed = currentTime;
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    double fieldTeleopTime = 140.0 - DriverStation.getMatchTime();

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;

    } else if (DriverStation.isEnabled()) {

      if (Math.abs(fieldTeleopTime - currentTime) >= timeResetThreshold
          && fieldTeleopTime <= 135
          && DriverStation.isFMSAttached()) {
        shiftTimerOffset += currentTime - fieldTeleopTime;
        currentTime = timerValue + shiftTimerOffset;
      }

      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }

      if (currentShiftIndex < 0) {
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // Combine adjacent shifts if same state
      if (currentShiftIndex > 0 &&
          currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
        stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
      }

      if (currentShiftIndex < shiftEndTimes.length - 1 &&
          currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
        stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }

    return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] schedule = getSchedule();

    double[] shiftedStart = new double[shiftStartTimes.length];
    double[] shiftedEnd = new double[shiftEndTimes.length];

    for (int i = 0; i < shiftStartTimes.length; i++) {
      if (schedule[i]) {
        // Active period → shift earlier
        shiftedStart[i] = shiftStartTimes[i] + shootEarlyOffset;
        shiftedEnd[i] = shiftEndTimes[i] + shootLateOffset;
      } else {
        // Inactive period → leave unchanged
        shiftedStart[i] = shiftStartTimes[i];
        shiftedEnd[i] = shiftEndTimes[i];
      }
    }

    return getShiftInfo(schedule, shiftedStart, shiftedEnd);
  }
}