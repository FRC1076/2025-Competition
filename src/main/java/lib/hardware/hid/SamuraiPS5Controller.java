// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.hid;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SamuraiPS5Controller extends CommandPS5Controller {
    public static double kDefaultStickDeadband = 0.02;

    private double stickDeadband;

    private DoubleSupplier leftStickX_DB;
    private DoubleSupplier leftStickY_DB;
    private DoubleSupplier rightStickX_DB;
    private DoubleSupplier rightStickY_DB;

    public SamuraiPS5Controller(int port) {
        this(port, kDefaultStickDeadband);
    }

    public SamuraiPS5Controller(int port, double stickDeadband) {
        super(port);
        this.stickDeadband = stickDeadband;
        configSticks();
    }

    /** Yes we are fancy */
    public SamuraiPS5Controller withDeadband(double deadband) {
        stickDeadband = deadband;
        configSticks();
        return this;
    }

    private void configSticks() {
        leftStickX_DB = () -> MathUtil.applyDeadband(super.getLeftX(), stickDeadband);
        leftStickY_DB = () -> MathUtil.applyDeadband(super.getLeftY(), stickDeadband);
        
        rightStickX_DB = () -> MathUtil.applyDeadband(super.getRightX(), stickDeadband);
        rightStickY_DB = () -> MathUtil.applyDeadband(super.getRightY(), stickDeadband);
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
}
