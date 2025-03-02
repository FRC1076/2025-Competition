// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

/**
 * Allows for a dynamic slew rate defined by a DoubleSupplier, rather than a constant double
 */
public class DynamicSlewRateLimiter {
    private final DoubleSupplier positiveRateLimitSupplier;
    private final DoubleSupplier negativeRateLimitSupplier;
    private double m_prevVal;
    private double m_prevTime;

    //TODO: ADD JAVADOC COMMENTS

    public DynamicSlewRateLimiter(DoubleSupplier positiveRateLimitSupplier, DoubleSupplier negativeRateLimitSupplier, double initialValue) {
        this.positiveRateLimitSupplier = positiveRateLimitSupplier;
        this.negativeRateLimitSupplier = negativeRateLimitSupplier;
        m_prevVal = initialValue;
        m_prevTime = MathSharedStore.getTimestamp();
    }

    public DynamicSlewRateLimiter(DoubleSupplier rateLimitSupplier, double initialValue) {
        this(
            rateLimitSupplier,
            () -> -rateLimitSupplier.getAsDouble(),
            initialValue
        );
    }

    /**
     * Creates a new DynamicSlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    public DynamicSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this(
            () -> positiveRateLimit,
            () -> negativeRateLimit,
            initialValue
        );
    }

    /**
     * Creates a new DynamicSlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public DynamicSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal +=
            MathUtil.clamp(
                input - m_prevVal,
                negativeRateLimitSupplier.getAsDouble() * elapsedTime,
                positiveRateLimitSupplier.getAsDouble() * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Returns the value last calculated by the SlewRateLimiter.
     *
     * @return The last value.
     */
    public double lastValue() {
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = MathSharedStore.getTimestamp();
    }
}