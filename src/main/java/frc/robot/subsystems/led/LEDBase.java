// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// DO NOT DELETE COMMENTS
// THEY ARE FOR EDUCATIONAL PURPOSES
package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants.LEDStates;

/** This is an interface so you can define what functionality you want
 * without defining how it actually happens.
 */
public interface LEDBase {
    /** 
     * Set the state of the LEDs.
     * <p>
     * This is "abstract" in the interface because all implementations must have it.
     * If this were set to default instead,
     * it would be optional to write an overriding implemetation.
     * <p>
     * This method has no curly braces because the implementation will be different
     * each time. It would have empty curly braces if it were default instead of abstract.
     * 
     * @param state The state of the LEDs found in Constants.LEDConstants.LEDStates
     */
    public abstract void setState(LEDStates state);
    public default LEDStates getState(){
        return null;
    }
}
