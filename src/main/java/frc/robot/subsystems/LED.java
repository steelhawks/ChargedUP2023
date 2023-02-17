package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDColor;

public class LED extends SubsystemBase {

  private AddressableLED LEDStrip;
  private AddressableLEDBuffer LEDBuffer;
  private static int rainbowStart = 0;
  private double lastChange;
  private boolean isOn;

  public LED(int port, int length) {
    LEDStrip = new AddressableLED(port);
    LEDBuffer = new AddressableLEDBuffer(length);

    LEDStrip.setLength(LEDBuffer.getLength());
   
    LEDStrip.setData(LEDBuffer);
    LEDStrip.start();
  }

  public void setColor(LEDColor color) {

    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, color.r, color.g, color.b);
    }

    LEDStrip.setData(LEDBuffer);
  }

  public void pulse(LEDColor color, double interval) {
    double timestamp = Timer.getFPGATimestamp();

    if (timestamp - lastChange > interval) {
      lastChange = timestamp;
      isOn = !isOn;
    }

    if (isOn) {
      stop();
    }
    else {
      setColor(color);
    }
  }

  public void rainbow() {

    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      i %= LEDBuffer.getLength();

      final var hue = (rainbowStart + (i * 180 / LEDBuffer.getLength())) % 180;
      LEDBuffer.setHSV(i, hue, 255, 128);
    }

    LEDStrip.setData(LEDBuffer);

    rainbowStart += 3;
    rainbowStart %= 180;
  }

  // public void wave() {
  //   for (int i = 0; i < LEDBuffer.getLength(); i++) {
  //     LEDBuffer.getLED(i).blue

  //     final var hue = (rainbowStart + (i * 180 / LEDBuffer.getLength())) % 180;
  //     LEDBuffer.setHSV(i, hue, 255, 128);
  //   }

  //   LEDStrip.setData(LEDBuffer);

  //   rainbowStart += 3;
  //   rainbowStart %= 180;
  // }

  public void stop() {
    setColor(LEDColor.OFF);
  }


  @Override
  public void periodic() {}
}