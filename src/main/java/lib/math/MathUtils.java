// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.math;

import java.math.BigInteger;

public final class MathUtils {

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

    /** returns the factorial of n */
    public static long Factorial(long n) {
        return n == 0 ? 1 : n * Factorial(n - 1);
    }

    public static long Factorial(int n) {
        return Factorial((long) n);
    }

    public static BigInteger BigFactorial(BigInteger n) {
        if (n.equals(BigInteger.valueOf(0))) {
            return BigInteger.valueOf(1);
        } else {
            return n.multiply(BigFactorial(n.subtract(BigInteger.valueOf(1))));
        }
    }
}
