// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  
  private final Alert motorDisconectedAlert;

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.21, 0.0, 3.07); //Null pointer exeption if we go back to this control scheme

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
  //** Consistant but very slow */
  // public void setL1Coral(){
  //   io.setPosition(new ExponentialProfile.State(8, 0));
  // }

  // /** Set position to score L2 CORAL */
  // public void setL2Coral(){
  //   io.setPosition(new ExponentialProfile.State(0, 0));
  // }

  // /** Set position to score L3 CORAL */
  // public void setL3Coral(){
  //   io.setPosition(new ExponentialProfile.State(16, 0));
  // }

  // /** Set position to colract from Coral Station */
  // public void setCoralStation(){
  //   io.setPosition(new ExponentialProfile.State(20, 0));
  // }

  //**Almost too fast going up nearly coasts on the way down; still accurate */
  // public void setL1Coral(){
  //   io.setPosition(8, feedforward.calculate(0));
  // }

  // /** Set position to score L2 CORAL */
  // public void setL2Coral(){
  //   io.setPosition(0, feedforward.calculate(0));
  // }

  // /** Set position to score L3 CORAL */
  // public void setL3Coral(){
  //   io.setPosition(16, feedforward.calculate(0));
  // }

  // /** Set position to colract from Coral Station */
  // public void setCoralStation(){
  //   io.setPosition(20, feedforward.calculate(0));
  // }

  public void setL1Coral(){
    io.setPosition(8);
  }

  /** Set position to score L2 CORAL */
  public void setL2Coral(){
    io.setPosition(0);
  }

  /** Set position to score L3 CORAL */
  public void setL3Coral(){
    io.setPosition(16);
  }

  /** Set position to colract from Coral Station */
  public void setCoralStation(){
    io.setPosition(20);
  }
}
