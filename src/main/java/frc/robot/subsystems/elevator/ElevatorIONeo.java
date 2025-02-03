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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorIONeo implements ElevatorIO{
    //4:1 gearbox
    private final SparkMax neo = new SparkMax(6, MotorType.kBrushless); //TODO Set device ID
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder encoder = neo.getEncoder();

    private final SparkClosedLoopController controller = neo.getClosedLoopController();
    
    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1, 1); //TODO Tune values
    private final ExponentialProfile profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(10, 1, 1)); //TODO change kV and kA to match feedforward
    private ExponentialProfile.State desiredState = new ExponentialProfile.State(0, 0);
    private ExponentialProfile.State setpoint = new ExponentialProfile.State(0, 0);

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
            .pid(0.492, 0.0, 0) //TODO Tune and change PID values
            .outputRange(0, 4); //TODO Change output range to fit gear ratio
        
        neo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.isConnected = motorConnectedDebouncer.calculate(neo.getLastError() == REVLibError.kOk);
        inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());

        inputs.appliedVolts = neo.getBusVoltage();
        inputs.currentAmps = neo.getOutputCurrent();
    }

    @Override
    public void setPosition(double outputRads, double feedforward){
        controller.setReference(Units.radiansToRotations(outputRads), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
    }

    @Override
    public void setPosition(ExponentialProfile.State desiredState){
        this.desiredState = desiredState;
        ExponentialProfile.State next = profile.calculate(0.02, setpoint, desiredState);

        controller.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(next.velocity) / 12.0);
        //? Can I change the 12.0 to affect speed?
        setpoint = next;
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
