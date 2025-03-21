// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;

public class CoralIONeo550 implements CoralIO{
    private SparkMax leftNeo550 = new SparkMax(52, MotorType.kBrushless); //TODO Change motor IDs
    private SparkMax rightNeo550 = new SparkMax(53, MotorType.kBrushless);

    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    private SparkClosedLoopController leftController = leftNeo550.getClosedLoopController();
    private SparkClosedLoopController rightController = rightNeo550.getClosedLoopController();

    private RelativeEncoder leftEncoder = leftNeo550.getEncoder();
    private RelativeEncoder rightEncoder = rightNeo550.getEncoder();

    private Debouncer leftConnectedDebouncer;
    private Debouncer rightConnectedDebouncer;

    public CoralIONeo550(){
        leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12.0);
        leftConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        leftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1, 0, 0)
            .outputRange(-1, 1); //TODO Change to fit with gear ratio

        rightConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12.0);
        rightConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        rightConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1, 0, 0)
            .outputRange(-1, 1); //TODO Change to fit with gear ratio
        
        leftNeo550.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightNeo550.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftConnectedDebouncer = new Debouncer(0.5);
        rightConnectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(CoralInputs inputs){
        inputs.leftIsConnected = leftConnectedDebouncer.calculate(leftNeo550.getLastError() == REVLibError.kOk);
        inputs.rightIsConnected = rightConnectedDebouncer.calculate(rightNeo550.getLastError() == REVLibError.kOk);

        inputs.leftVelocityRotPerSec = leftEncoder.getVelocity();
        inputs.rightVelocityRotPerSec = rightEncoder.getVelocity();

        inputs.leftAppliedVolts = leftNeo550.getBusVoltage();
        inputs.rightAppliedVolts = rightNeo550.getBusVoltage();

        inputs.leftCurrentAmps = leftNeo550.getOutputCurrent();
        inputs.rightCurrentAmps = rightNeo550.getOutputCurrent();

        inputs.averageVelocityRPM = (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0;
        inputs.averageAppliedVolts = (leftNeo550.getBusVoltage() + rightNeo550.getBusVoltage()) / 2.0;
        inputs.averageCurrentAmps = (leftNeo550.getOutputCurrent() + rightNeo550.getOutputCurrent()) / 2.0;
    }

    @Override
    public void setVelocity(double velocityRPM){
        leftController.setReference(velocityRPM, ControlType.kVelocity);
        rightController.setReference(velocityRPM, ControlType.kVelocity);
    }

    @Override
    public void setVelocity(boolean isRightMotor, double velocityRPM){
        if (isRightMotor){
            rightController.setReference(velocityRPM, ControlType.kVelocity);
        } else {
            leftController.setReference(velocityRPM, ControlType.kVelocity);
        }
    }

    @Override
    public void setVelocity(double leftVelocityRPM, double rightVelocityRPM){
        leftController.setReference(leftVelocityRPM, ControlType.kVelocity);
        rightController.setReference(rightVelocityRPM, ControlType.kVelocity);
    }

    @Override
    public void setSpeed(double speed){
        leftNeo550.set(speed);
        rightNeo550.set(speed);
    }

    @Override
    public void setSpeed(double leftSpeed, double rightSpeed){
        leftNeo550.set(leftSpeed);
        rightNeo550.set(rightSpeed);
    }

    @Override
    public void setBreakMode(boolean enable){
        leftConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        rightConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void stop(){
        leftNeo550.stopMotor();
        rightNeo550.stopMotor();
    }
}

