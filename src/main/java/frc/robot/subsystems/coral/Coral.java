// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  private final CoralIO io;
  private final CoralInputsAutoLogged inputs = new CoralInputsAutoLogged();
  
  private final Alert leftMotorDisconnected;
  private final Alert rightMotorDisconnected;

  public Coral(CoralIO io) {
    this.io = io;
    io.setBreakMode(false);

    io.updateInputs(inputs);

    leftMotorDisconnected = new Alert("Left coral motor disconnected. System may not function properly.", AlertType.kError);
    rightMotorDisconnected = new Alert("Right coral motor disconnected. System my not function properly.", AlertType.kError);

    leftMotorDisconnected.set(!inputs.leftIsConnected);
    rightMotorDisconnected.set(!inputs.rightIsConnected);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Coral", inputs);
  }

  /** Run motors to score L2 and L3 CORAL */
  public void scoreCoral(){
    // io.setVelocity(3850); //TODO Find and set velocity needed to score L2 and L3
    io.setSpeed(0.35);
    //? Are L2 and L3 velocities the same?
  }

  /** Run motors at different velocities to spit into L1(Trough) */
  public void scoreTrough(){
    io.setVelocity(0, 0); //TODO Find and set velocities to score L1
  }

  /** Stop the motors */
  public void stop(){
    io.stop();
  }

  //? Is this something we want? Or will the coral just rest until we fire it?
  /** Run motors to intake coral */
  public void intake(){
    io.setVelocity(0); //TODO Find and set velocities to intake without spitting out
    //? What logic can be added to stop the intake when there is a coral inside?
  }
}
