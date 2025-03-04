// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// DO NOT DELETE COMMENTS
// THEY ARE FOR EDUCATIONAL PURPOSES
package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LEDStates;

/** This is kind of like a subsystem.
 * <p>
 * The implementation to be used will be selected in the constructor upon instantiation.
 * <p>
 * All of the methods in this file will call the corresponding method in the chosen IO layer.
 */
public class LEDSubsystem extends SubsystemBase{
    private final LEDBase io;
    private LEDStates previousState = LEDStates.IDLE;

    /** Create the LEDs with one of the IO layers.
     * 
     * @param io The chosen IO layer.
     */
    public LEDSubsystem(LEDBase io) {
        this.io = io;
    }

    /** Set the state of the LEDs through the chosen IO layer.
     * 
     * @param state The chosen state in the enum LEDStates.
     */
    public void setState(LEDStates state) {
        this.previousState = this.io.getState();
        this.io.setState(state);
    }

    private Command updateBuilder(boolean isAutoAligned, boolean safeToMoveElevator, boolean safeToFeedCoral) {
        if (isAutoAligned) {
            return setTempStateTimed(LEDStates.AUTO_ALIGNED, 2.0);
        } else if (safeToFeedCoral) {
            return Commands.runOnce(() -> this.setState(LEDStates.HUMAN_PLAYER_CAN_DROP));
        } else if (safeToMoveElevator) {
            return Commands.runOnce(() -> this.setState(LEDStates.CORAL_INDEXED));
        } else {
            return Commands.runOnce(() -> this.setState(LEDStates.IDLE));
        }
    }

    public Command update(BooleanSupplier isAutoAligned, BooleanSupplier safeToMoveElevator, BooleanSupplier safeToFeedCoral) {
        return new DeferredCommand(() -> updateBuilder(isAutoAligned.getAsBoolean(),safeToMoveElevator.getAsBoolean(),safeToFeedCoral.getAsBoolean()), null);
    }
    
    /** Sets the state of the LEDs through the chosen IO layer,
     * and then reverts the LEDs to the IDLE state.
     * 
     * @param state The state to apply to the LEDs
     * @param seconds The number of seconds to wait before reverting to the IDLE state
      */
    public Command setStateTimed(LEDStates state, double seconds) {
        return Commands.startEnd(
            () -> setState(state),
            () -> setState(LEDStates.IDLE),
            this
        ).withTimeout(seconds);
    }

    /**
     * Sets the state of the LEDs through the chosen IO layer,
     * and then reverts the LEDs to the previous state.
     * 
     * @param state The state to apply to the LEDs
     * @param seconds The number of seconds to wait before reverting to the previous state
     */
    public Command setTempStateTimed(LEDStates state, double seconds) {
        return Commands.startEnd(
            () -> {
                setState(state);
            },
            () -> setState(this.previousState),
            this
        ).withTimeout(seconds);
    }
}
