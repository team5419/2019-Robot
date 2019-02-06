package frc.util;

import frc.robot.RobotMap;

/**
 * natural disasters unit conversion class
 */
public class NDiUnits {
    /**
     * ratio of inches to feet
     */
    public static double inchesToFeet = 12;

    /**
     * Convert inches to feet.
     * @param inches inches to be convert
     * @return feet
     */
    public static double inchesToFeet(double inches) {
        return inches * inchesToFeet;

    }

    /**
     * ratio of inches to encoder ticks
     */
    public static double inchesToEncoderTicks = 1f / RobotMap.driveTrainWheelCircumference * RobotMap.driveTrainTicksPerRevelution;

    /**
     * Convert inches to encoder ticks.
     * @param inches inches to be convert
     * @return encoder ticks
     */
    public static double inchesToEncoderTicks(double inches) {
        return inches * inchesToEncoderTicks;
    }

    /**
     * ratio of inches to meters
     */
    public static double inchesToMeters = 0.0254f;

    /**
     * Convert inches to meters.
     * @param inches inches to be convert
     * @return meters
     */
    public static double inchesToMeters(double inches) {
        return inches * inchesToMeters;
    }

    /**
     * ratio of feet to inches
     */
    public static double feetToInches = 1f / 12f;

    /**
     * Convert feet to inches.
     * @param feet feet to be convert
     * @return inches
     */
    public static double feetToInches(double feet) {
        return feet * feetToInches;
    }

    /**
     * ratio of feet to meters
     */
    public static double feetToMeters = 0.3048f;

    /**
     * Convert feet to meters.
     * @param feet feet to be convert
     * @return meters
     */
    public static double feetToMeters(double feet) {
        return feet * feetToMeters;
    }

    /**
     * ratio of feet to encoder ticks
     */
    public static double feetToEncoderTicks = inchesToEncoderTicks * inchesToFeet;

    /**
     * Convert feet to encoder ticks.
     * @param feet feet to be convert
     * @return encoder ticks
     */
    public static double feetToEncoderTicks(double feet) {
        return feet * feetToEncoderTicks;
    }

    /* METERS CONVERSION */

    /**
     * ratio of meters to inches
     */
    public static double metersToInches = 1f / inchesToMeters;

    /**
     * Convert meters to inches.
     * @param meters meters to be convert
     * @return inches
     */
    public static double metersToInches(double feet) {
        return feet * metersToInches;
    }

    /**
     * ratio of meters to feet
     */
    public static double metersToFeet = 1f / feetToMeters;

    /**
     * Convert meters to feet.
     * @param meters meters to be convert
     * @return feet
     */
    public static double metersToFeet(double feet) {
        return feet * metersToFeet;
    }

    /**
     * ratio of meters to encoder ticks
     */
    public static double metersToEncoderTicks = inchesToEncoderTicks * inchesToMeters;

    /**
     * Convert meters to encoder ticks.
     * @param meters meters to be convert
     * @return encoder ticks
     */
    public static double metersToEncoderTicks(double feet) {
        return feet * metersToEncoderTicks;
    }

    /**
     * ratio of encoder ticks to inches
     */
    public static double encoderTicksToInches = 1f / inchesToEncoderTicks;

    /**
     * Convert encoder ticks to inches.
     * @param encoderTicks encoder ticks to be convert
     * @return inches
     */
    public static double encoderTicksToInches(double encoderTicks) {
        return encoderTicks * encoderTicksToInches;
    }

    /**
     * ratio of encoder ticks to feet
     */
    public static double encoderTicksToFeet = 1f / feetToEncoderTicks;

    /**
     * Convert encoder ticks to feet.
     * @param encoderTicks encoder ticks to be convert
     * @return feet
     */
    public static double encoderTicksToFeet(double encoderTicks) {
        return encoderTicks * encoderTicksToFeet;
    }

    /**
     * ratio of encoder ticks to meters
     */
    public static double encoderTicksToMeters = 1f / metersToEncoderTicks;

    /**
     * Convert encoder ticks to meters.
     * @param encoderTicks encoder ticks to be convert
     * @return meters
     */
    public static double encoderTicksToMeters(double encoderTicks) {
        return encoderTicks * encoderTicksToMeters;
    }

    /**
     * ratio of encoder seconds to tenths of a second (100ms)
     */
    public static double secondsToTenths = 10;

    /**
     * Convert seconds to tenths of a second (100ms).
     * @param encoderTicks seconds to be convert
     * @return tenths (100ms)
     */
    public static double secondsToTenths(double seconds) {
        return seconds * secondsToTenths;
    }

    /**
     * ratio of tenths of a second (100ms) to seconds
     */
    public static double tenthsToSeconds = 1f / 10f;

    /**
     * Convert tenths of a second (100ms) to seconds.
     * @param encoderTicks tenths of a second (100ms) to be convert
     * @return seconds
     */
    public static double tenthsToSeconds(double tenths) {
        return tenths * tenthsToSeconds;
    }

    /* MULTI CONVERSION */

    /**
     * Convert a compond unit to a new type using {@paramref  a} / {@paramref  b}.
     * @param a the conversion to perform on numerator
     * @param b the conversion to perform on denominator
     * @param units to units to convers
     * @return converted units
     */
    public static double compoundConvsion(double a, double b, double units) {
        return units * a / b;
    }
}
