// DO NOT DELETE COMMENTS
// THEY ARE FOR EDUCATIONAL PURPOSES
package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;

import edu.wpi.first.wpilibj.DigitalOutput;

// TODO: name things better
/** This is a class that actually does (implements) the stuff in LEDIO.java */
public class LEDIODigitalPins implements LEDBase {
    /* Declare an object for each pin */
    private final DigitalOutput m_pin1;
    private final DigitalOutput m_pin2;
    private final DigitalOutput m_pin3;

    /** Instantiate each digital pin, using integer constants for the channel */
    public LEDIODigitalPins() {
        m_pin1 = new DigitalOutput(LEDConstants.kDIOPort1);
        m_pin2 = new DigitalOutput(LEDConstants.kDIOPort2);
        m_pin3 = new DigitalOutput(LEDConstants.kDIOPort3);
    }

    /** Overrides the method in LEDIO.java so it actually does something.
     * <p>
     * This implementation is specifically if you are using digital pins.
     */
    @Override
    public void setState(LEDStates state) {
        // LEDStates is an enum, and state is an instance of that enum,
        // and .onesPlace is a boolean element of that enum.
        m_pin1.set(state.onesPlace);
        m_pin2.set(state.twosPlace);
        m_pin3.set(state.foursPlace);
    }
}
