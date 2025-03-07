// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// DO NOT DELETE COMMENTS
// THEY ARE FOR EDUCATIONAL PURPOSES
package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.LEDConstants.LEDState;
import frc.robot.utils.VirtualSubsystem;

/** This is kind of like a subsystem.
 * <p>
 * The implementation to be used will be selected in the constructor upon instantiation.
 * <p>
 * All of the methods in this file will call the corresponding method in the chosen IO layer.
 */
public class LEDSubsystem extends VirtualSubsystem {

    private final LEDBase io;
    private boolean autoAlignFlag = false; // Whether or not an auto-align routine is running
    private boolean disabledFlag = false; // Whether or not the robot code is disabled
    private boolean elevatorZeroingFlag = false; // Whether or not the elevator is being zeroed
    private boolean hpSignalFlag = false; // Whether or not the driver is signalling the human player
    private boolean coralIndexedFlag = false;


    /** Create the LEDs with one of the IO layers.
     * 
     * @param io The chosen IO layer.
     */
    public LEDSubsystem(LEDBase io) {
        this.io = io;
    }

    private LEDState calculateStateFromFlags() {
        if (disabledFlag) {
            return LEDState.OFF;
        }

        if (autoAlignFlag) {
            return LEDState.AUTO_ALIGNING;
        }

        if (elevatorZeroingFlag) {
            return LEDState.ZEROING_ELEVATOR;
        }

        return LEDState.IDLE;
    }

    private void calculateAndUpdateState() {
        io.setState(calculateStateFromFlags());
    }

    public void setAutoAlignFlag(boolean flag) {
        autoAlignFlag = flag;
        calculateAndUpdateState();
    }

    public void setDisabledFlag(boolean flag) {
        disabledFlag = flag;
        calculateAndUpdateState();
    }

    public void setElevatorZeroingFlag(boolean flag) {
        elevatorZeroingFlag = flag;
        calculateAndUpdateState();
    }

    public void resetFlags() {
        autoAlignFlag = false;
        disabledFlag = false;
        elevatorZeroingFlag = false;
        calculateAndUpdateState();
    }
    /** Set the state of the LEDs through the chosen IO layer.
     * 
     * @param state The chosen state in the enum LEDStates.
     *
    public void setState(LEDState state) {
        this.io.setState(state);
    }
    */

    @Override
    public void periodic() {
        io.periodic();
    }

    /** Enables autoAlign flag at initialization, disables flag on command end */
    public Command applyAutoAlignFlag() {
        return Commands.startEnd(
            () -> setAutoAlignFlag(true), 
            () -> setAutoAlignFlag(false)
        );
    }

    public Command applyElevatorZeroingFlag() {
        return Commands.startEnd(
            () -> setElevatorZeroingFlag(true),
            () -> setElevatorZeroingFlag(false)
        );
    }

    public Command applyDisabledFlag() {
        return Commands.startEnd(
            () -> setDisabledFlag(true),
            () -> setDisabledFlag(false)
        );
    }

    public Command applyFlagReset() {
        return Commands.runOnce(() -> resetFlags());
    }

    
}
