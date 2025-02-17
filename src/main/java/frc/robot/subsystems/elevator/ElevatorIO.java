// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.Height;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public boolean isConnected = false;
        public double positionRads = 0.0;
        public double velocityRadPerSec = 0.0;

        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public Height currentElevatorSetpoint = Height.HOME;
    }

    /** Updates the set of loggable inputs 
     * @param inputs set of updateable inputs
     */
    public default void updateInputs(ElevatorIOInputs inputs){}

    /** Run motor to specified position with open loop
     * @param output desired hight of the elevator in inches
     */
    public default void setPosition(double output){}

    /** Run motor to specified position with open loop
     * @param height desired setpoint of the elevator
     */
    public default void setPosition(Height height){}

    /** Run motor at specified voltage 
     * @param outputVolts deired voltage to run the motor
     */
    public default void runVoltage(double outputVolts){}

    /** Run motor at specified speed
     * @param speed deired speed of the motor (-1 - 1)
     */
    public default void setSpeed(double speed){}

    /** Set brake mode enabled 
     * @param enable weather to enable break mode
     */
    public default void setBrakeMode(boolean enable){}

    /** Stop the motor */
    default void stop(){}
} 
