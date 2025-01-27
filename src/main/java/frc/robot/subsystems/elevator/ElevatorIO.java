// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public boolean isConnected = false;
        public double positionRads = 0.0;
        public double velocityRadPerSec = 0.0;

        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(ElevatorIOInputs inputs){}

    /** Run motor at specified open loop value */
    public default void setPosition(double outputRads, double feedforward){}

    /** Run motor at specified voltage */
    public default void runVoltage(double outputVolts){}

    /** Run motor at specified speed (-1 to 1) */
    public default void setSpeed(double speed){}

    /** Set brake mode enabled */
    public default void setBrakeMode(boolean enable){}

    /** Stop the motor */
    default void stop(){}
} 
