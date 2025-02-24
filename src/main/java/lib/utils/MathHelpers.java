// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.utils;

public final class MathHelpers {

    /**
     * Clamps a value between a minimum and maximum value.
     * @param value The value to clamp.
     * @param min The minimum value.
     * @param max The maximum value.
     * @return The clamped value.
     */
    public static double clamp(double value, double min, double max) {
        if(min > max) {
            throw new IllegalArgumentException("min must be less than max");
        }

        return Math.max(min, Math.min(max, value));
    }
}
