package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDOnRIOConstants;
import frc.robot.Constants.LEDConstants.LEDStates;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDOnRIO implements LEDBase {
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_buffer;

    // LED patterns
    private final LEDPattern solidPurple = LEDPattern
        .solid(Color.kPurple)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));
    private final LEDPattern flashingPurple = LEDPattern
        .solid(Color.kPurple)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));
    private final LEDPattern flashingWhite = LEDPattern
        .solid(Color.kWhite)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));
    private final LEDPattern flashingGreen = LEDPattern
        .solid(Color.kGreen)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));
    private final LEDPattern rainbow = LEDPattern
        .rainbow(255, 255)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .scrollAtRelativeSpeed(Percent.per(Seconds).of(25));


    public LEDOnRIO() {
        m_leds = new AddressableLED(LEDOnRIOConstants.kPWMPort);
        m_buffer = new AddressableLEDBuffer(LEDOnRIOConstants.kLength);

        // Setting the length is intensive, so ONLY update data after this
        m_leds.setLength(m_buffer.getLength());

        solidPurple.applyTo(m_buffer); // Start at solid purple
        m_leds.setData(m_buffer);
        m_leds.start();
    }

    @Override
    public void setState(LEDStates state) {
        if(state == LEDStates.EMPTY) {
            // Solid purple
            solidPurple.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.CORAL_INDEX) {
            // Flashing purple
            flashingPurple.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.CORAL_GRABBER) {
            // Flashing white
            flashingWhite.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.ALGAE) {
            // Flashing green
            flashingGreen.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        }
    }
}