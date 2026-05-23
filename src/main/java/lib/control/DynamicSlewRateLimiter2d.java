// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

/**
 * Allows for a dynamic slew rate defined by a DoubleSupplier, rather than a constant double
 * <p> This class deals with 2d inputs, in order to limit the acceleration vector in a 2d space, like a robot field.
 */
public class DynamicSlewRateLimiter2d {
    private final DoubleSupplier positiveRateLimitSupplier;
    private final DoubleSupplier negativeRateLimitSupplier;
    private double m_prevValX;
    private double m_prevValY;
    private double m_prevTimeX;
    private double m_prevTimeY;

    /**
     * A class that limits the rate of change of an input value. 
     * Useful for implementing voltage, setpoint, and/or output ramps. 
     * A slew-rate limit is most appropriate when the quantity being controlled is a velocity or a voltage; 
     * when controlling a position, consider using a TrapezoidProfile instead.
     * <p> This slew rate limiter is dynamic so that the rate limit can be changed on the fly, due to changing conditions, like the center of gravity
     * <p> This class deals with 2d inputs, in order to limit the acceleration vector in a 2d space, like a robot field.
     * @param positiveRateLimitSupplier
     * @param negativeRateLimitSupplier
     * @param initialValue
     */
    public DynamicSlewRateLimiter2d(DoubleSupplier positiveRateLimitSupplier, DoubleSupplier negativeRateLimitSupplier, double initialValueX, double initialValueY) {
        this.positiveRateLimitSupplier = positiveRateLimitSupplier;
        this.negativeRateLimitSupplier = negativeRateLimitSupplier;
        m_prevValX = initialValueX;
        m_prevValY = initialValueY;
        m_prevTimeX = m_prevTimeY = MathSharedStore.getTimestamp();
    }

    /**
     * A class that limits the rate of change of an input value. 
     * Useful for implementing voltage, setpoint, and/or output ramps. 
     * A slew-rate limit is most appropriate when the quantity being controlled is a velocity or a voltage; 
     * when controlling a position, consider using a TrapezoidProfile instead.
     * <p> This slew rate limiter is dynamic so that the rate limit can be changed on the fly, due to changing conditions, like the center of gravity
     * <p> This class deals with 2d inputs, in order to limit the acceleration vector in a 2d space, like a robot field.
     * @param rateLimitSupplier
     * @param initialValue
     */
    public DynamicSlewRateLimiter2d(DoubleSupplier rateLimitSupplier, double initialValue) {
        this(
            rateLimitSupplier,
            () -> -rateLimitSupplier.getAsDouble(),
            initialValue,
            initialValue
        );
    }

    /**
     * Creates a new DynamicSlewRateLimiter2d with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    public DynamicSlewRateLimiter2d(double positiveRateLimit, double negativeRateLimit, double initialValueX, double initialValueY) {
        this(
            () -> positiveRateLimit,
            () -> negativeRateLimit,
            initialValueX,
            initialValueY
        );
    }

    /**
     * Creates a new DynamicSlewRateLimiter2d with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public DynamicSlewRateLimiter2d(double rateLimit) {
        this(rateLimit, -rateLimit, 0, 0);
    }

    /**
     * Filters the x input to limit its slew rate.
     *
     * @param inputX The x input value whose slew rate is to be limited.
     * @param inputY The y input value whose slew rate is to be limited.
     */
    public double calculateX(double inputX, double inputY) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - m_prevTimeX;
        
        // Calculate the change in x and y since the last calculation
        double changeX = inputX - m_prevValX;
        double changeY = inputY - m_prevValY;

        // Calculate the angle and magnitude of the change
        double changeAngleRadians = Math.atan2(changeY, changeX);
        double changeMagnitude = Math.sqrt(changeX * changeX + changeY * changeY);

        // Clamp the magnitude of the change to the slew rate limit
        changeMagnitude = MathUtil.clamp(
            changeMagnitude,
            negativeRateLimitSupplier.getAsDouble() * elapsedTime,
            positiveRateLimitSupplier.getAsDouble() * elapsedTime);

        // Update the x and y values, scaling the change by the new magnitude
        m_prevValX += changeMagnitude * Math.cos(changeAngleRadians);

        m_prevTimeX = currentTime;

        return m_prevValX;
    }

    /**
     * Filters the y input to limit its slew rate.
     *
     * @param inputX The x input value whose slew rate is to be limited.
     * @param inputY The y input value whose slew rate is to be limited.
     */
    public double calculateY(double inputX, double inputY) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - m_prevTimeY;
        
        // Calculate the change in x and y since the last calculation
        double changeX = inputX - m_prevValX;
        double changeY = inputY - m_prevValY;

        // Calculate the angle and magnitude of the change
        double changeAngleRadians = Math.atan2(changeY, changeX);
        double changeMagnitude = Math.sqrt(changeX * changeX + changeY * changeY);

        // Clamp the magnitude of the change to the slew rate limit
        changeMagnitude = MathUtil.clamp(
            changeMagnitude,
            negativeRateLimitSupplier.getAsDouble() * elapsedTime,
            positiveRateLimitSupplier.getAsDouble() * elapsedTime);

        // Update the x and y values, scaling the change by the new magnitude
        m_prevValY += changeMagnitude * Math.sin(changeAngleRadians);

        m_prevTimeY = currentTime;

        return m_prevValY;
    }

    /**
     * Returns the x value last calculated by the SlewRateLimiter.
     *
     * @return The last value.
     */
    public double lastValueX() {
        return m_prevValX;
    }

    /**
     * Returns the y value last calculated by the SlewRateLimiter.
     *
     * @return The last value.
     */
    public double lastValueY() {
        return m_prevValY;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double valueX, double valueY) {
        m_prevValX = valueX;
        m_prevValY = valueY;
        m_prevTimeX = MathSharedStore.getTimestamp();
        m_prevTimeY = MathSharedStore.getTimestamp();
    }
}