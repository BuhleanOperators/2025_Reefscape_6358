// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.BracePosition;

/** Add your docs here. */
public interface BraceIO {

    @AutoLog
    public class BraceInputs{
        public boolean isConnected = false;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates set of loggable inputs 
     * @param inputs The set of updatable inputs
     */
    public default void updateInputs(BraceInputs inputs){}

    /** Runs the motor at specified speed
     * @param speed The desired speed of the motor (-1 to 1)
     */
    public default void runSpeed(double speed){}

    /** Runs the motor to the specified position
     * @param position The desired position of the motor
     */
    public default void setPosition(BracePosition position){}

    /** Stops the motor*/
    public default void stop(){}
}
