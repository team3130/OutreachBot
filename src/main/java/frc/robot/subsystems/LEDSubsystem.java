// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;


public class LEDSubsystem extends SubsystemBase {
  private final int ledLength = 61;
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  DigitalInput limitSwitch;
  private double hue = 0;
  private double sat = 0;
  private double value = 0;
  private int rainbowFirstPixelHue = 20;
  private int firstPixelColor = 20;


  /** Creates a new ExampleSubsystem. */
  public LEDSubsystem() {
    /// PWM port 9
    // Must be a PWM header, not MXP or DIO
    led = new AddressableLED(1);
    limitSwitch = new DigitalInput(0);


    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(ledLength);
    led.setLength(ledBuffer.getLength());


    // Set the data
    led.setData(ledBuffer);
    led.start();
  }


  // HSV: h == set the value, s == saturation, v == brightness value
  // Red falls between 0 and 60 degrees.
  // Yellow falls between 61 and 120 degrees.
  // Green falls between 121 and 180 degrees.
  // Cyan falls between 181 and 240 degrees.
  // Blue falls between 241 and 300 degrees.
  // Magenta falls between 301 and 360 degrees.


  public void red() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setHSV(i, 0, 255, 128);
    }
    led.setData(ledBuffer);
  }


  public void green() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setHSV(i, 50, 255, 128);
    }
    led.setData(ledBuffer);
  }


  public void blue() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setHSV(i, 80, 255, 128);
    }
    led.setData(ledBuffer);
  }


  public void reset() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setHSV(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }


  public void setCustom(int h, int s, int v) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setHSV(i, h, s, v);
    }
    led.setData(ledBuffer);
  }


  public void staticRainbow() {
    int firstPixelHue = 20;
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (firstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    led.setData(ledBuffer);
  }


  public void movingRainbow() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    led.setData(ledBuffer);
  }

  public void random() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      ledBuffer.setHSV(i, (int) (180 * Math.random()), (int) (255 * Math.random()), 128);
    }
  }

  public boolean hitLimitSwitch() {
    return !limitSwitch.get();
  }


  public double getHue() {
    return hue;
  }


  public double getSat() {
    return sat;
  }
  public double getVal() {
    return value;
  }


  public void setHue(double h) {
    hue = h;
  }


  public void setSat(double s) {
    sat = s;
  }


  public void setVal(double v) {
    value = v;
  }


  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LEDSubsystem");
    builder.addDoubleProperty("Hue", this::getHue, this::setHue);
    builder.addDoubleProperty("Saturation", this::getSat, this::setSat);
    builder.addDoubleProperty("Value", this::getVal, this::setVal);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}