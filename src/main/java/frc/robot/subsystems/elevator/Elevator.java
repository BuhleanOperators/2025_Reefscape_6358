// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Height;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  
  private final Alert motorDisconectedAlert;

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

  /** Stop the motor */
  public void stop(){
    io.stop();
  }

  /** Set the position of the elevator
   * @param height The desired state of the elevator
   */
  public void setPosition(Height height){
    io.setPosition(height);
  }
}
