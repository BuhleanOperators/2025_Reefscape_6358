// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LedIORio implements LedIO{
    private static final int length = 0; //TODO change length
    private static final int centerLed = 95;
    private static final int halfLength = (int) Math.ceil(length / 2.0);

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public LedIORio(){
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void setMode(LEDMode mode){
        switch (mode){
            case DISABLED_NEUTRAL:
                solid(Color.kDarkOrange);
                break;

            case DISABLED_BLUE:
                solid(Color.kBlue);
                break;

            case DISABLED_RED:
                solid(Color.kRed);
                break;

            case INAKE_ALGAE:
                break;

            case EXTAKE_ALGAE:
                break;

            case TROUGH_CORAL:
                break;

            case BRANCH_CORAL:
                break;

            case PARTIAL_CLIMB:
                break;

            case FULL_CLIMB:
                break;

            case CORAL_READY:
                break;

            case CORAL_WAIT:
                break;
            
            case AUTO:
                break;
            
            default:
                solid(Color.kBlack);
                break;
        }
    }

    //----- LED Pattern Methods -----
    private void solid(Color color){
        for (int i = 0; i < length; i++){
            buffer.setLED(i, color);
        }
    }

    private void strobe(Color color, double duration){
        boolean on = 
            ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(on ? color : color.kBlack);
    }
    
    private void breathe(Color color1, Color color2, double duration){
        double x = 
            ((Timer.getFPGATimestamp() % duration) / duration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        double red = (color1.red * (1 - ratio)) + (color2.red * ratio);
        double green = (color1.green * (1 - ratio)) + (color2.green * ratio);
        double blue = (color1.blue * (1 - ratio)) + (color2.blue * ratio);
        solid(new Color(red, green, blue));
    }

    private void wave(Color c1, Color c2, double fullLength, double duration, double waveExponent) {
        double x = 
            (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / fullLength;
        for (int i = 0; i < halfLength; i++) {
          x += xDiffPerLed;
          double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
          if (Double.isNaN(ratio)) {
            ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
          }
          if (Double.isNaN(ratio)) {
            ratio = 0.5;
          }
          double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
          double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
          double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
          setLedsSymmetrical(i, new Color(red, green, blue));
        }
      }

      private void setLedsSymmetrical(int index, Color color) {
        buffer.setLED((centerLed + index) % length, color);
        buffer.setLED(centerLed - index, color);
      }
}
