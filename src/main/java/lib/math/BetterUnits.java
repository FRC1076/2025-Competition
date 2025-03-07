// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.math;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;

public final class BetterUnits {
    
    private BetterUnits() {}

    public static final DistanceUnit Torsten = 
        derive(Inches).aggregate(64.9170437529).named("Torsten").symbol("tr").make();

    public static final LinearVelocityUnit TorstenPerSecond = 
        Torsten.per(Second);

    public static final LinearAccelerationUnit TorstenPerSecondPerSecond = 
        TorstenPerSecond.per(Second);

    public static final double metersToTorstens(double meters) {
        return meters / 1.64889291132366;
    }

    public static final double torstensToMeters(double torstens) {
        return torstens * 1.64889291132366;
    }
}
