// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeDirection;
import frc.robot.Constants.algaeSpeed;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  private Alert motorDisconnectedAlert;

  public Algae(AlgaeIO io) {
    this.io = io;
    io.setBreakMode(false);

    io.updateInputs(inputs);

    motorDisconnectedAlert = new Alert("Algea motor disconnected. System may not function properly.", AlertType.kError);
    motorDisconnectedAlert.set(!inputs.isConnected);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }

  /** Grab the algea */
  public void grabAlgae(){
    io.setSpeed(algaeSpeed.intakeSpeed);//TODO Set speed needed to grab algae
  }

  /** Oextake the algea */
  public void extakeAlgae(){
    io.setSpeed(algaeSpeed.extakeSpeed);//TODO Set speed needed to eject algae
  }

  /** Stop the motor */
  public void stop(){
    io.stop();
  }

  public void run(){
    switch (Constants.currentDirection){
      case INTAKE:
        grabAlgae();
        Constants.currentDirection = AlgaeDirection.EXTAKE;
        break;
      case EXTAKE:
        extakeAlgae();
        Constants.currentDirection = AlgaeDirection.INTAKE;
        break;
      default:
        break;
    }
  }
}
