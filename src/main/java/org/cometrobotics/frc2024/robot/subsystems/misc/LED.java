package org.cometrobotics.frc2024.robot.subsystems.misc;

import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

/**
 * Controls LED lighting using a CANdle device from CTRE Phoenix, with methods for
 * setting colors and configuring the LEDs.
 * @see https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html
 */
public class LED extends SubsystemBase {

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

    public Command setColor(Supplier<Color8Bit> colorSupplier) {
        return Commands.runOnce(
            () -> {
                Color8Bit color = colorSupplier.get();
                candle.setLEDs(color.red, color.green, color.blue);
            },
            this
        );
    }
}
