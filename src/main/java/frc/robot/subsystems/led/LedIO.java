// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

/** Add your docs here. */
public interface LedIO {

    public default void setMode(LEDMode mode){}

    public static enum LEDMode{
        DISABLED_RED, DISABLED_BLUE, DISABLED_NEUTRAL,
        INAKE_ALGAE, EXTAKE_ALGAE,
        TROUGH_CORAL, BRANCH_CORAL,
        PARTIAL_CLIMB, FULL_CLIMB,
        CORAL_READY, CORAL_WAIT,
        AUTO,
        DEFALUT
    }
}
