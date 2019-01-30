package frc.util;

import frc.robot.RobotMap;

/**
 * natural disasters unit conversion class
 */
public class NDiUnits {
    /**
     * ratio of inchs to feet
     */
    public static double inchsToFeet = 12;
    public static double inchsToFeet(double inchs) {
        return inchs * inchsToFeet;
    }

    /**
     * ratio of inchs to encoder ticks
     */
    public static double inchsToEncoderTicks = 1f / RobotMap.driveTrainWheelCircumference * RobotMap.driveTrainTicksPerRevelution;
    public static double inchsToEncoderTicks(double inchs) {
        return inchs * inchsToEncoderTicks;
    }

    /**
     * ratio of inchs to meters
     */
    public static double inchsToMeters = 0.0254f;
    public static double inchsToMeters(double inchs) {
        return inchs * inchsToMeters;
    }

    /**
     * ratio of feet to inchs
     */
    public static double feetToInchs = 1f / 12f;
    public static double feetToInchs(double feet) {
        return feet * feetToInchs;
    }

    /**
     * ratio of feet to meters
     */
    public static double feetToMeters = 0.3048f;
    public static double feetToMeters(double feet) {
        return feet * feetToMeters;
    }

    /**
     * ratio of feet to encoder ticks
     */
    public static double feetToEncoderTicks = inchsToEncoderTicks * inchsToFeet;
    public static double feetToEncoderTicks(double feet) {
        return feet * feetToEncoderTicks;
    }

    /* METERS CONVERSION */

    /**
     * ratio of meters to inchs
     */
    public static double metersToInchs = 1f / inchsToMeters;
    public static double metersToInchs(double feet) {
        return feet * metersToInchs;
    }

    /**
     * ratio of meters to feet
     */
    public static double metersToFeet = 1f / feetToMeters;
    public static double metersToFeet(double feet) {
        return feet * metersToFeet;
    }

    /**
     * ratio of meters to encoder ticks
     */
    public static double metersToEncoderTicks = inchsToEncoderTicks * inchsToMeters;
    public static double metersToEncoderTicks(double feet) {
        return feet * metersToEncoderTicks;
    }

    /**
     * ratio of encoder ticks to inchs
     */
    public static double encoderTicksToInchs = 1f / inchsToEncoderTicks;
    public static double encoderTicksToInchs(double encoderTicks) {
        return encoderTicks * encoderTicksToInchs;
    }

    /**
     * ratio of encoder ticks to feet
     */
    public static double encoderTicksToFeet = 1f / feetToEncoderTicks;
    public static double encoderTicksToFeet(double encoderTicks) {
        return encoderTicks * encoderTicksToFeet;
    }

    /**
     * ratio of encoder ticks to meters
     */
    public static double encoderTicksToMeters = 1f / metersToEncoderTicks;
    public static double encoderTicksToMeters(double encoderTicks) {
        return encoderTicks * encoderTicksToMeters;
    }

    /**
     * ratio of encoder seconds to tenths of a second (100ms)
     */
    public static double secondsToTenths = 10;
    public static double secondsToTenths(double seconds) {
        return seconds * secondsToTenths;
    }

    /**
     * ratio of tenths of a second (100ms) to seconds
     */
    public static double tenthsToSeconds = 1f / 10f;
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
