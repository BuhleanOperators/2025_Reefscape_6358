// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import frc.robot.Constants.BracePosition;
import frc.robot.Constants.braceSetpoints;

/** Add your docs here. */
public class BraceIO_PG71 implements BraceIO{
    private final SparkMax motor = new SparkMax(6, MotorType.kBrushed); //TODO Change CAN ID
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    
    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

    public BraceIO_PG71(){
        config
            .inverted(true) //TODO Check if inverted
            .idleMode(IdleMode.kBrake)   
            .smartCurrentLimit(40) //Amps 
            .voltageCompensation(12.0);

        config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1.0)
            .countsPerRevolution(8192);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1, 0.0, 0.0) //TODO Tune and change PID values
            .outputRange(-1, 1) 
            .maxMotion
                .allowedClosedLoopError(0.01)
                .maxAcceleration(75)
                .maxVelocity(75);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(BraceInputs inputs){
        inputs.isConnected = motorConnectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk);
        inputs.appliedVolts = motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void runSpeed(double speed){
        motor.set(speed);
    }

    @Override
    public void setPosition(BracePosition position){
        switch (position){
            case HOME:
                controller.setReference(braceSetpoints.home, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0);
                Constants.currentPosition = BracePosition.HOME;
                break;

            case PARTIAL:
                controller.setReference(braceSetpoints.partial, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0);
                Constants.currentPosition = BracePosition.PARTIAL;
                break;
            
            case FULL:
                controller.setReference(braceSetpoints.full, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0);
                Constants.currentPosition = BracePosition.FULL;
                break;
            
            default:
                break;
        }
    }

    @Override
    public void stop(){
        motor.stopMotor();
    }
}
