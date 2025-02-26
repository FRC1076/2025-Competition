// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {

    private static List<VirtualSubsystem> subsystems = new ArrayList<>();

    public VirtualSubsystem() {
        subsystems.add(this);
    }

    public static void periodicAll() {
        for (var subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public abstract void periodic();
}
