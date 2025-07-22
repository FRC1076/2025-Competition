package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDOnRIOConstants;
import frc.robot.Constants.LEDConstants.LEDStates;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

public class LEDOnRIO implements LEDBase {
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_buffer;

    /* LED patterns */
    private final LEDPattern solidPurple = LEDPattern
        .solid(Color.kPurple)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));
    
        private final LEDPattern solidWhite = LEDPattern
        .solid(Color.kWhite)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));

    private final LEDPattern solidGreen = LEDPattern
        .solid(Color.kGreen)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));

    private final LEDPattern solidOrange = LEDPattern
        .solid(Color.kOrange)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));
    
    private final LEDPattern solidRed = LEDPattern
        .solid(Color.kRed)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));

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
        .scrollAtRelativeSpeed(Percent.per(Seconds).of(100));

        private final LEDPattern rainbowFlash = LEDPattern
        .rainbow(255, 255)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds))
        .scrollAtRelativeSpeed(Percent.per(Seconds).of(100));

    private final LEDPattern off = LEDPattern.kOff;

    // Flashing color states for HP signals
    private final LEDPattern flashingRed = LEDPattern
        .solid(Color.kRed)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));

    private final LEDPattern flashingOrange = LEDPattern
        .solid(Color.kOrange)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));

    private final LEDPattern flashingYellow = LEDPattern
        .solid(Color.kYellow)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));

    // Flashing green already exists

    private final LEDPattern flashingBlue = LEDPattern
        .solid(Color.kMediumBlue)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));

    private final LEDPattern flashingPurple = LEDPattern
        .solid(Color.kPurple)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));


    public LEDOnRIO() {
        m_leds = new AddressableLED(LEDOnRIOConstants.kPWMPort);
        m_buffer = new AddressableLEDBuffer(LEDOnRIOConstants.kLength);

        // Setting the length is intensive, so ONLY update data after this
        m_leds.setLength(m_buffer.getLength());

        off.applyTo(m_buffer); // Start at solid purple
        m_leds.setData(m_buffer);
        m_leds.start();
    }

    @Override
    public void setState(LEDStates state) {
        if (state == LEDStates.IDLE) {
            // Solid purple
            solidPurple.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.CORAL_INDEXED) {
            solidRed.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.AUTO_ALIGNING) {
            rainbow.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.AUTO_ALIGNED) {
            rainbowFlash.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.ELEVATOR_ZEROED) {
            solidOrange.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.HUMAN_PLAYER_SIGNAL) {
            flashingGreen.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.RED_HP_SIGNAL) {
            flashingRed.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.ORANGE_HP_SIGNAL) {
            flashingOrange.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.YELLOW_HP_SIGNAL) {
            flashingYellow.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.GREEN_HP_SIGNAL) {
            flashingGreen.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.BLUE_HP_SIGNAL) {
            flashingBlue.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.PURPLE_HP_SIGNAL) {
            flashingPurple.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.OFF) {
            off.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        }
    }
}