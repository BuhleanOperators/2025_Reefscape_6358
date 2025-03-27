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
  public static Height currentHeight = Height.HOME;
  public static AlgaeDirection currentDirection = AlgaeDirection.INTAKE;
  public static boolean isCompetition = false; //TODO change comp mode

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum Height {
    HOME,
    L2,
    L3, 
    HIGH_ALGAE
  }

  public enum AlgaeDirection {
    INTAKE,
    EXTAKE
  }

  public class elevatorHeight{
    public static double L1 = 0;
    public static double L2 = 8;
    public static double L3 = 24.5;
    public static double highAlgae = 19;
  }

  public class coralSpeed{
    public static double troughLeft = 0.15;
    public static double troughRight = 0.45;
    public static double speed = 0.45;
    public static double reverseSpeed = 0.1;
  }

  public class algaeSpeed{
    public static double intakeSpeed = 1.0;
    public static double extakeSpeed = -1.0;
  }
}