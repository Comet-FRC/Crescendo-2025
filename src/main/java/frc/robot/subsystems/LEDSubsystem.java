package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @see https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html
 */
public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle;

    public LEDSubsystem(int deviceID) {
        candle = new CANdle(deviceID);
        configureCANdle();

        setColor(255, 255, 255);
    }

    void configureCANdle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        candle.configAllSettings(config);
    }

    public void setColor(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }
}
