// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.trajectory.ExponentialProfile;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public boolean isConnected = false;
        public double positionRads = 0.0;
        public double velocityRadPerSec = 0.0;

        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs 
     * @param inputs set of updateable inputs
     */
    public default void updateInputs(ElevatorIOInputs inputs){}

    /** Run motor at specified open loop value 
     * @param outputRads desired position of the elevator in radians
     * @param feedforward feedforward controller for feedack control
     */
    public default void setPosition(double outputRads, double feedforward){}

    /** Run motor to specified position with open loop
     * @param desiredState exponential profile state (use inches for position and 0 as velocity)
     */
    public default void setPosition(ExponentialProfile.State desiredState){}

    /** Run motor to specified position with open loop
     * @param output desired hight of the elevator in inches
     */
    public default void setPosition(double output){}

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
