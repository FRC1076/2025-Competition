package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDOnRIOConstants;
import frc.robot.Constants.LEDConstants.LEDState;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.EnumMap;
import java.util.Map;

//TODO: ADD OFF STATE

public class LEDOnRIO implements LEDBase {
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_buffer;

    // STATIC DATA
    /* LED patterns */
    private static final LEDPattern solidPurple = LEDPattern
        .solid(Color.kPurple)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));

    private static final LEDPattern solidGreen = LEDPattern
        .solid(Color.kGreen)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));

    private static final LEDPattern solidOrange = LEDPattern
        .solid(Color.kOrange)
        .atBrightness(Percent.of(LEDOnRIOConstants.kEmptyStateBrightness));

    private static final LEDPattern flashingPurple = LEDPattern
        .solid(Color.kPurple)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));

    private static final LEDPattern flashingWhite = LEDPattern
        .solid(Color.kWhite)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));

    private static final LEDPattern flashingGreen = LEDPattern
        .solid(Color.kGreen)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));

    private static final LEDPattern flashingDarkBlue = LEDPattern
        .solid(Color.kDarkBlue)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .blink(Seconds.of(LEDOnRIOConstants.kFlashSeconds));

    private static final LEDPattern rainbow = LEDPattern
        .rainbow(255, 255)
        .atBrightness(Percent.of(LEDOnRIOConstants.kFlashingStateBrightness))
        .scrollAtRelativeSpeed(Percent.per(Seconds).of(100));

    private static final LEDPattern off = LEDPattern.kOff;


    private static final Map<LEDState,LEDPattern> patternMap = new EnumMap<>(LEDState.class);

    static {
        patternMap.put(LEDState.IDLE,solidPurple);
        patternMap.put(LEDState.CORAL_INDEXED,flashingPurple);
        patternMap.put(LEDState.HUMAN_PLAYER_SIGNAL,flashingGreen);
        patternMap.put(LEDState.ALGAE,flashingDarkBlue);
        patternMap.put(LEDState.AUTO_ALIGNING,rainbow);
        patternMap.put(LEDState.ZEROING_ELEVATOR,solidOrange);
        patternMap.put(LEDState.OFF,off);
    }
    

    public LEDOnRIO() {
        m_leds = new AddressableLED(LEDOnRIOConstants.kPWMPort);
        m_buffer = new AddressableLEDBuffer(LEDOnRIOConstants.kLength);

        // Setting the length is intensive, so ONLY update data after this
        m_leds.setLength(m_buffer.getLength());

        solidPurple.applyTo(m_buffer); // Start at solid purple
        m_leds.start();
    }

    @Override
    public void setState(LEDState state) {
        var statePattern = patternMap.get(state);
        statePattern.applyTo(m_buffer);
    }

    @Override
    public void periodic() {
        m_leds.setData(m_buffer);
    }
}