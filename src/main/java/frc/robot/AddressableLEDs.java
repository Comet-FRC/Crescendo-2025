package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AddressableLEDs {

    private static AddressableLED m_led = new AddressableLED(0);
    private static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

    public static void hasNoteColor(AddressableLED led, AddressableLEDBuffer ledBuffer){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
            m_ledBuffer.setHSV(i, 0, 100, 100);
        }
        led.setData(ledBuffer);
    }

    public static void defaultColor(AddressableLED led, AddressableLEDBuffer ledBuffer){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 52, 29, 92);
            m_ledBuffer.setHSV(i, 0, 100, 100);
        }
        led.setData(ledBuffer);
    }
    
    private static void rainbow() {
        int m_rainbowFirstPixelHue = 0;
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }
}
