// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.hid;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SamuraiXboxController extends CommandXboxController {
    public static double kDefaultStickDeadband = 0.02;
    public static double kDefaultTriggerThreshold = 0.7;

    private double stickDeadband;
    private double triggerThreshold;

    private DoubleSupplier leftStickX_DB;
    private DoubleSupplier leftStickY_DB;
    private DoubleSupplier rightStickX_DB;
    private DoubleSupplier rightStickY_DB;

    public SamuraiXboxController(int port) {
        this(port,kDefaultStickDeadband,kDefaultTriggerThreshold);
    }

    public SamuraiXboxController(int port, double stickDeadband, double triggerThreshold) {
        super(port);
        this.stickDeadband = stickDeadband;
        this.triggerThreshold = triggerThreshold;
        configSticks();
    }

    /** Yes we are fancy */
    public SamuraiXboxController withDeadband(double deadband) {
        stickDeadband = deadband;
        configSticks();
        return this;
    }

    /** Yes we are fancy */
    public SamuraiXboxController withTriggerThreshold(double threshold) {
        triggerThreshold = threshold;
        return this;
    }

    private void configSticks() {
        leftStickX_DB = () -> MathUtil.applyDeadband(super.getLeftX(),stickDeadband);
        leftStickY_DB = () -> MathUtil.applyDeadband(super.getLeftY(),stickDeadband);
        
        rightStickX_DB = () -> MathUtil.applyDeadband(super.getRightX(),stickDeadband);
        rightStickY_DB = () -> MathUtil.applyDeadband(super.getRightY(),stickDeadband);
    }

    @Override
    public double getLeftX() {
        return leftStickX_DB.getAsDouble();
    }

    @Override
    public double getLeftY() {
        return leftStickY_DB.getAsDouble();
    }

    @Override
    public double getRightX() {
        return rightStickX_DB.getAsDouble();
    }

    @Override
    public double getRightY() {
        return rightStickY_DB.getAsDouble();
    }

    @Override
    public Trigger leftTrigger() {
        return super.leftTrigger(triggerThreshold);
    }

    @Override
    public Trigger rightTrigger() {
        return super.rightTrigger(triggerThreshold);
    }

}
