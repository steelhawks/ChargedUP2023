package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LEDColor;

public class LED extends SubsystemBase {
    private AddressableLED LEDStrip;
    private AddressableLEDBuffer LEDBuffer;
    private int waveIndex;
    private static int start = 0;

    public LED() {
        this.waveIndex = 0;
        this.LEDStrip = new AddressableLED(0);
        this.LEDBuffer = new AddressableLEDBuffer(60);

        this.LEDStrip.setLength(this.LEDBuffer.getLength());
        this.LEDStrip.setData(this.LEDBuffer);
        this.LEDStrip.start();
    }

    public void setColor(LEDColor color) {
        for (byte i = 0; i < this.LEDBuffer.getLength(); i++) {
            this.LEDBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        }

        this.LEDStrip.setData(this.LEDBuffer);
    }

    public void wave(int waveLength, LEDColor color) {
        int red = color.getRed();
        int green = color.getGreen();
        int blue = color.getBlue();

        if (this.waveIndex == 0) {
            for (byte i = 0; i <= waveLength; i++) {
                this.LEDBuffer.setRGB(i, red, green, blue);
            }
        } else {
            for (byte i = 0; i < this.LEDBuffer.getLength() - 2; i++) {
                if (this.waveIndex > 0) this.LEDBuffer.setRGB(this.waveIndex - 1, 0, 0, 0);

                if (this.waveIndex + waveLength < LEDBuffer.getLength() - 1) {
                    this.LEDBuffer.setRGB(this.waveIndex + waveLength, red, green, blue);
                } else {
                    this.LEDBuffer.setRGB(this.waveIndex + waveLength - 59, red, green, blue);
                }

                if (this.waveIndex > LEDBuffer.getLength() - 1) this.waveIndex = -1;
            }
        }
        waveIndex += 1;

        this.LEDStrip.setData(this.LEDBuffer);
    }

    public void rainbow(){
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            final var hue = (start + (i * 180 / LEDBuffer.getLength())) % 180;
            LEDBuffer.setHSV(i, hue, 255, 128);
       }
       LEDStrip.setData(LEDBuffer);
       start %= 180;
       start +=5 ;
     }

    public void endgameLED(double time) {
        if (time > 105  && time < 110) {
            this.rainbow();
        }
        if (time > 110 && time < 110.05) {
            this.setColor(new LEDColor(0, 0, 0));
        }
    }
}
