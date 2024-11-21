package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

/**
 * @see https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html
 */
public class LED {

    /* Singleton */
	
	private static LED instance = null;
	
	public static LED getInstance() {
		if (instance == null) instance = new LED();
		return instance;
	}

	/* Implementation */

    private final CANdle candle;

    private LED() {
        candle = new CANdle(21);
        configureCANdle();

        //setColor(255, 255, 255);
        RainbowAnimation anim = new RainbowAnimation(1, 0.5, 68);
        candle.animate(anim);
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
