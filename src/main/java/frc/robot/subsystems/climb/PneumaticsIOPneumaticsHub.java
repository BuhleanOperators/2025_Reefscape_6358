// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

/** Add your docs here. */
public class PneumaticsIOPneumaticsHub implements PneumaticsIO{
    private PneumaticHub hub = new PneumaticHub(10);
    private DoubleSolenoid piston = hub.makeDoubleSolenoid(8, 10);
    private Compressor compressor = hub.makeCompressor();

    public PneumaticsIOPneumaticsHub(){
    }

    @Override
    public void updateInputs(PneumaticsInputs inputs){
        inputs.compressorIsEnabled = compressor.isEnabled();
        inputs.pistionPosition = piston.get();
        inputs.pressure = compressor.getPressure();
    }

    @Override
    public void setPosition(DoubleSolenoid.Value position){
        piston.set(position);
    }

    @Override
    public void togglePosition(){
        piston.toggle();
    }

    @Override
    public void enableCompressor(){
        hub.enableCompressorAnalog(100, 120);
    }
}
