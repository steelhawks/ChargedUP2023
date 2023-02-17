package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LEDColor;

public class LED extends SubsystemBase {
    private AddressableLED LEDStrip;
    private AddressableLEDBuffer LEDBuffer;
    private int waveIndex = 0;

    public LED() {
        this.LEDStrip = new AddressableLED(0);
        // Each LED in the LED strip
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

        if (waveIndex == 0) {
            for (byte i = 0; i <= waveLength; i++) {
                this.LEDBuffer.setRGB(i, red, green, blue);
            }
        } else {
            for (byte i = 0; i < this.LEDBuffer.getLength(); i++) {
                if (waveIndex > 0) {
                    this.LEDBuffer.setRGB(waveIndex - 1, red, green, blue);
                }
                if (waveIndex + waveLength < 59) {
                    this.LEDBuffer.setRGB(waveIndex + waveLength, red, green, blue);
                } else {
                    this.LEDBuffer.setRGB(waveIndex + waveLength - 59, red, green, blue);
                }
                if (waveIndex > 59) {
                    waveIndex = -1;
                }
            }
        }
        waveIndex += 1;

        this.LEDStrip.setData(this.LEDBuffer);
    }
}
