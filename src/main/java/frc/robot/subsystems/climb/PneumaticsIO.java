// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public interface PneumaticsIO {

    @AutoLog
    public static class PneumaticsInputs {
        public boolean compressorIsEnabled = false;
        public DoubleSolenoid.Value pistionPosition = Value.kOff;
        public double pressure = 0.0;
    }

    /** Updates set of loggable inputs 
     * @param inputs The set of updatable inputs
     */
    public default void updateInputs(PneumaticsInputs inputs){}
    
    /** Sets the state of the piston to the specified state
     * @param position The desired position of the piston
     */
    public default void setPosition(DoubleSolenoid.Value position){}

    /** Toggles the state of the piston */
    public default void togglePosition(){}

    /** Enables the compressor */
    public default void enableCompressor(){}
}
