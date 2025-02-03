// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  
  private final Alert motorDisconectedAlert;

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0); //Null pointer exeption if we go back to this control scheme

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);

    motorDisconectedAlert = new Alert("Elevator motor disconnected.", AlertType.kError);

    motorDisconectedAlert.set(!inputs.isConnected);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Set position of elevator to specified setpoint with feedforward control 
   * @param pos the distance the elevator should stop at in inches from zero
   */
  private void setPosition(double pos){
    TrapezoidProfile.State setpointRads = new State(pos, 0);
    double ff = feedforward.calculate(setpointRads.position, setpointRads.velocity);
    io.setPosition(setpointRads.position, ff);
  }

  /** Set speed of elevator motor to specified speed
   * @param speed desired speed of the motor (-1 - 1)
   */ 
  public void setSpeed(double speed){
    io.setSpeed(speed);
  }

  /** Stop the motor */
  public void stop(){
    io.stop();
  }

  // //TODO Find and set positions for each needed level
  // //!TEST VALUES FOR MOVING CHAIN ONLY
  /** Set position to score L1 CORAL */
  public void setL1Coral(){
    io.setPosition(new ExponentialProfile.State(8, 0));
  }

  /** Set position to score L2 CORAL */
  public void setL2Coral(){
    io.setPosition(new ExponentialProfile.State(0, 0));
  }

  /** Set position to score L3 CORAL */
  public void setL3Coral(){
    io.setPosition(new ExponentialProfile.State(16, 0));
  }

  /** Set position to colract from Coral Station */
  public void setCoralStation(){
    io.setPosition(new ExponentialProfile.State(20, 0));
  }
}
