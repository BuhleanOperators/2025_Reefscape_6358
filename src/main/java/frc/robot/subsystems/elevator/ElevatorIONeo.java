// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.elevatorSetpoints;

/** Add your docs here. */
public class ElevatorIONeo implements ElevatorIO{
    //4:1 gearbox
    private final SparkMax neo = new SparkMax(6, MotorType.kBrushless); 
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder encoder = neo.getEncoder();

    private final SparkClosedLoopController controller = neo.getClosedLoopController();
    
    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

    public ElevatorIONeo(){
        config
            .inverted(true) 
            .idleMode(IdleMode.kBrake)   
            .smartCurrentLimit(40) //Amps
            .voltageCompensation(12.0);

        config.encoder
            .positionConversionFactor(1.01)
            .velocityConversionFactor(1.0);
        
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.61, 0.0, 0.0) //TODO Tune and change PID values
            .outputRange(-1, 1) 
            .maxMotion
                .allowedClosedLoopError(0.25)
                .maxAcceleration(5000)
                .maxVelocity(5000);
        
        neo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.isConnected = motorConnectedDebouncer.calculate(neo.getLastError() == REVLibError.kOk);
        inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());

        inputs.appliedVolts = neo.getBusVoltage();
        inputs.currentAmps = neo.getOutputCurrent();

        inputs.currentElevatorSetpoint = Constants.currentHeight;
    }

    @Override
    public void setPosition(ElevatorHeight height){
        switch (height){
            case HOME:
                controller.setReference(elevatorSetpoints.L1, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.8);
                Constants.currentHeight = ElevatorHeight.HOME;
                break;
            
            case L2:
                controller.setReference(elevatorSetpoints.L2, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.8);
                Constants.currentHeight = ElevatorHeight.L2;
                break;
        
            case L3:
                controller.setReference(elevatorSetpoints.L3, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.8);
                Constants.currentHeight = ElevatorHeight.L3;
                break;

            default:
                break;
        }
    }

    @Override
    public void runVoltage(double outputVolts){
        neo.setVoltage(outputVolts);
    }

    @Override
    public void setSpeed(double speed){
        neo.set(speed);
    }

    @Override
    public void setBrakeMode(boolean enable){
        config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void stop(){
        neo.stopMotor();
    }

    public double getMotorPositionRads(){
        return Units.rotationsToRadians(encoder.getPosition());
    }

    public SparkClosedLoopController getPIDController(){
        return controller;
    }

    public void resetEncoder(){
        encoder.setPosition(0.0);
    }

}
