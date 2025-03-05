// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class Led{
    //Robot state tracking
    public boolean hpAttentionAlert = false;
    public boolean endgameAlert = false;
    public boolean lowBatteryAlert = false;
    public boolean partialClimb = false;
    public boolean fullClimb = false;
    public boolean intakeAlgae = false;
    public boolean extakeAlgae = false;

    private Optional<Alliance> alliance = Optional.empty();
    private Color disabledColor = Color.kDarkOrange;
    private Color secondaryDisabledColor = Color.kBlack;
    private boolean lastEnabledAuto = false;
    private double lastEnabledTime = 0.0;
    private boolean estopped = false;

    //Constants
    private static final int length = 32; //TODO Change length
    private static final Section fullSection = new Section(0, length);
    private static final Section swerveModulesSection = new Section(0, 0); //TODO set Section values
    private static final Section elevatorSection = new Section(0, 0);
    private static final Section funnelSection = new Section(0, 0);
    private static final double strobeDuration = 0.1;
    private static final double breathFastDuration = 0.5;
    private static final double breathSlowDuration = 1.0;
    private static final double rainbowCycleLength = 25.0;
    private static final double rainbowDuration = 0.25;
    private static final double waveExponent = 0.4;
    private static final double waveFastCycleLength = 25.0;
    private static final double waveFastDuration = 0.25;
    private static final double waveDisabledCycleLength = 15.0;
    private static final double waveDisabledDuration = 2.0;
    private static final double autoFadeTime = 2.5; // 3s nominal
    private static final double autoFadeMaxTime = 5.0;
    private static final Color teamColor = new Color("fd5800");

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private Led() {
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    public synchronized void periodic() {
        //Update alliance color
        if (DriverStation.isFMSAttached()){
            alliance = DriverStation.getAlliance();
            disabledColor = 
                alliance
                    .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
                    .orElse(disabledColor);
            secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : secondaryDisabledColor;
        }

        //Update Auto state
        if (DriverStation.isEnabled()){
            lastEnabledAuto = DriverStation.isAutonomous();
            lastEnabledTime = Timer.getTimestamp();
        }

        //Update estop state
        if (DriverStation.isEStopped()){
            estopped = true;
        }

        //Select LED mode
        solid(fullSection, Color.kBlack); //Defalut to off
        if (estopped){
            solid(fullSection, disabledColor);//TODO come up with colors & patterns for everything
        }else if(DriverStation.isDisabled()){
            if (lastEnabledAuto && Timer.getTimestamp() - lastEnabledTime < autoFadeMaxTime){ 
                //Auto Fade
                wave(new Section(
                    0, 
                    (int)(length * (1 - ((Timer.getTimestamp() - lastEnabledTime) / autoFadeTime)))),
                    Color.kDarkOrange,
                    Color.kBlack,
                    waveFastCycleLength,
                    waveFastDuration);
            }else if (lowBatteryAlert){
                strobe(elevatorSection, disabledColor, breathFastDuration);
            }else{ //Default pattern
                wave(
                    fullSection,
                    disabledColor,
                    secondaryDisabledColor,
                    waveDisabledCycleLength,
                    waveDisabledDuration
                );
            }
        }else if(DriverStation.isAutonomous()){
            wave(fullSection, Color.kDarkOrange, Color.kBlack, waveFastCycleLength, waveFastDuration);
        }else {
            solid(fullSection, disabledColor);
        }

        if (partialClimb){
            strobe(elevatorSection, disabledColor, breathFastDuration);
        }
        if (fullClimb){
            breathe(elevatorSection, secondaryDisabledColor, disabledColor, breathFastDuration);
        }
        if (intakeAlgae){
            strobe(elevatorSection, disabledColor, breathFastDuration);
        }
        if (extakeAlgae){
            strobe(elevatorSection, disabledColor, breathFastDuration);
        }
        if (endgameAlert){
            strobe(elevatorSection, disabledColor, breathFastDuration);
        }
    }

    //----- LED Pattern Methods -----
    private void solid(Section section, Color color){
        for (int i = 0; i < length; i++){
            buffer.setLED(i, color);
        }
    }

    private void strobe(Section section, Color color, double duration){
        boolean on = 
            ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(section, on ? color : Color.kBlack);
    }
    
    private void breathe(Section section, Color color1, Color color2, double duration){
        double x = 
            ((Timer.getFPGATimestamp() % duration) / duration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        double red = (color1.red * (1 - ratio)) + (color2.red * ratio);
        double green = (color1.green * (1 - ratio)) + (color2.green * ratio);
        double blue = (color1.blue * (1 - ratio)) + (color2.blue * ratio);
        solid(section, new Color(red, green, blue));
    }

    private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
        double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = section.end() - 1; i >= section.start(); i--) {
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
          buffer.setLED(i, new Color(red, green, blue));
        }
    }

      private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
        int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
        for (int i = section.end() - 1; i >= section.start(); i--) {
            int colorIndex =
            (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
            colorIndex = colors.size() - 1 - colorIndex;
            buffer.setLED(i, colors.get(colorIndex));
        }
    }

    private static record Section(int start, int end) {}
}
