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

  public void setPosition(TrapezoidProfile.State setpointRads){
    double ff = feedforward.calculate(setpointRads.position, setpointRads.velocity);
    io.setPosition(setpointRads.position, ff);
  }
  //TODO Create and logic elevator setpoint methods for scoring L1, L2, and L3 and collect from CORAL Station
}
