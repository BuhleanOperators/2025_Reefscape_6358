// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private final PneumaticsIO io;
  private final PneumaticsInputsAutoLogged inputs = new PneumaticsInputsAutoLogged();
  public Pneumatics(PneumaticsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pneumatics", inputs);
  }

  /** Retract the pistons */
  public void climbUp(){
    io.setPosition(DoubleSolenoid.Value.kReverse);
  }

  /** Extend the pistons */
  public void climbDown(){
    io.setPosition(DoubleSolenoid.Value.kForward);
  }

  /** Toggle the pistons */
  public void toggle(){
    io.togglePosition();
  }

  public void enableCompressor(){
    io.enableCompressor();
  }
}
