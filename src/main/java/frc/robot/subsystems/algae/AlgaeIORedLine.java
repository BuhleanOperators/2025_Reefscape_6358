// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class AlgaeIORedLine implements AlgaeIO{
    private SparkMax neo550 = new SparkMax(54, MotorType.kBrushed); //TODO Change motor ID
    private SparkMaxConfig config = new SparkMaxConfig();

    private Debouncer motorConnectedDebouncer;

    public AlgaeIORedLine(){
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .voltageCompensation(12.0);
        
        neo550.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorConnectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs){
        inputs.isConnected = motorConnectedDebouncer.calculate(neo550.getLastError() == REVLibError.kOk);
    }

    @Override
    public void setSpeed(double speed){
        neo550.set(speed);
    }

    @Override
    public void stop(){
        neo550.stopMotor();
    }

    @Override
    public void setBreakMode(boolean enable){
        config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
