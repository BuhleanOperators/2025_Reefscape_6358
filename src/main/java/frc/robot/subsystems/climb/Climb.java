// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BracePosition;

public class Climb extends SubsystemBase {
  private Brace brace;
  private Pneumatics pneumatics;

  public Climb(PneumaticsIO pneumaticsIO, BraceIO braceIO) {
    brace = new Brace(braceIO);
    pneumatics = new Pneumatics(pneumaticsIO);
  }

  @Override
  public void periodic() {
  }

  private void setPartialBrace(){
    brace.setPosition(BracePosition.PARTIAL);
  }

  private void setFullBrace(){
    brace.setPosition(BracePosition.FULL);
  }

  public void climbPistons(){
    switch (Constants.currentPosition) {
      case FULL:
        pneumatics.climbDown();
        break;
    
      default:
        break;
    }
  }

  public void run(){
    switch (Constants.currentPosition) {
      case HOME:
        setPartialBrace();
        break;
    
      case PARTIAL:
        setFullBrace();
        break;
        
      default:
        break;
    }
  }

  public void enableCompressor(){
    pneumatics.enableCompressor();
  }

  public void setInitalPiston(){
    pneumatics.climbUp();
  }
}
