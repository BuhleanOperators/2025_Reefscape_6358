// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorInputs{
        public boolean leftIsConnected = false;
        public boolean rightIsConnected = false;

        public double rightVelocityRotPerSec = 0.0;
        public double leftVelocityRotPerSec = 0.0;

        public double rightAppliedVolts = 0.0;
        public double leftAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double leftCurrentAmps = 0.0;

        public double averageAppliedVolts = 0.0;
        public double averageCurrentAmps = 0.0;
        public double averageVelocityRPM = 0.0;
    }

    /** Updates set of loggable inputs 
     * @param inputs set of updatable inputs
    */
    public default void updateInputs(EndEffectorInputs inputs){}

    /** Sets velocity of both end effector motors to specified velocity
     * <p>Motors rotate in opposite directions</p>
     * @param velocityRPM desired velocity of the motors in rotations per minute
    */
    public default void setVelocity(double velocityRPM){}

    /** Sets velocity of specified motor to specified velocity
     * @param isRightMotor weather or not to run the right motor
     * @param velocityRPM deired velocity of the motor in rotations per minute
    */
    public default void setVelocity(boolean isRightMotor, double velocityRPM){}

    /** Sets velocity of motors to specified velocities 
     * <p>Motors rotate in oposite directions on defalut.</p>
     * @param leftVelocityRPM deired velocity of the left motor in rotations per minute
     * @param rightVelocityRPM deired velocity of the right motor in rotations per minute
    */
    public default void setVelocity(double leftVelocityRPM, double rightVelocityRPM){}

    /** Sets the speed of the motors to specified percent
     * <p>Motors spin in oppoisite directions
     * @param speed Desired speed of the motors (1 to -1)
     */
    public default void setSpeed(double speed){}

    /** Set break mode enabled 
     * @param enable desired enabled state of break mode for both motors
    */
    public default void setBreakMode(boolean enable){}

    /** Stops both motors */
    public default void stop(){}
} 
