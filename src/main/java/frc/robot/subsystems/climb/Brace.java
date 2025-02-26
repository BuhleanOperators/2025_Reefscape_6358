// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BracePosition;

public class Brace extends SubsystemBase {
  private final BraceIO io;
  private final BraceInputsAutoLogged inputs = new BraceInputsAutoLogged();

  private final Alert motorDisconnected;

  public Brace(BraceIO io) {
    this.io = io;

    motorDisconnected = new Alert("Brace motor disconnected. System may not function as intended.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Brace", inputs);

    motorDisconnected.set(!inputs.isConnected);
  }
  
  /** Run the motor at a specified speed
   * @param speed The desired speed of the motor (-1 to 1)
   */
  public void runSpeed(double speed){
    io.runSpeed(speed);
  }

  /** Run the motor to the desired position
   * @param position The desired position of the motor
   */
  public void setPosition(BracePosition position){
    io.setPosition(position);
  }
}
