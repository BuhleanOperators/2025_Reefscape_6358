// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

/** This class contains global configuration describing the current robot and runtime mode. */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final Mode currentMode = Mode.REAL;
  public static ElevatorHeight currentHeight = ElevatorHeight.HOME;
  public static BracePosition currentPosition = BracePosition.HOME;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum ElevatorHeight {
    HOME,
    L2,
    L3
  }

  public enum BracePosition {
    HOME,
    PARTIAL,
    FULL
  }

  public class elevatorSetpoints{
    public static double L1 = 0;
    public static double L2 = 8;
    public static double L3 = 16;
    public static double lowAlgae = 0;
    public static double highAlgae = 0;
  }

  public class coralSpeed{
    public static double troughLeft = 0.15;
    public static double troughRight = 0.45;
    public static double speed = 0.45;
  }

  public class braceSetpoints{
    public static double home = 0.0;
    public static double partial = 0.25;
    public static double full = 0.5;
  }
}