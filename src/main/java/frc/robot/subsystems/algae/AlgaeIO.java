// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface AlgaeIO {
    @AutoLog
    public class AlgaeIOInputs{
        public boolean isConnected = false;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of updateable inputs
     * @param inputs set of updatable inputs
     */
    public default void updateInputs(AlgaeIOInputs inputs){}

    /** Sets the speed of the motor to specified speed 
     * <p>Positive pulls algae in, negative ejects
     * @param speed desired speed of the motor (1 to -1)
    */
    public default void setSpeed(double speed){}

    /** Sets the velocity of the motor to specified velocity
     * <p>Positive pulls algae in, negative ejects
     * @param velocityRPM desired velocity of the robot in rotations per minute
     */
    public default void setVelocity(double velocityRPM){}

    /** Set break mode enabled 
     * @param enable desired enabled state of break mode for both motors
    */
    public default void setBreakMode(boolean enable){}

    /** Stops the motor */
    public default void stop(){}
}
