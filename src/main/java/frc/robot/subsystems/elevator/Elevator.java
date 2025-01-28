// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  
  private final Alert motorDisconectedAlert;

  private ElevatorFeedforward feedforward;

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

  /** Set position of elevator to specified setpoint with feedforward control */
  private void setPosition(TrapezoidProfile.State setpointRads){
    double ff = feedforward.calculate(setpointRads.position, setpointRads.velocity);
    io.setPosition(setpointRads.position, ff);
  }

  //TODO FInd and set positions for each needed level
  /** Set position to score L1 CORAL */
  public void setL1Coral(){
    setPosition(null);
  }

  /** Set position to score L2 CORAL */
  public void setL2Coral(){
    setPosition(null);
  }

  /** Set position to score L3 CORAL */
  public void setL3Coral(){
    setPosition(null);
  }

  /** Set position to colract from Coral Station */
  public void setCoralStation(){
    setPosition(null);
  }
}
