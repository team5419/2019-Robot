package frc.util;

import frc.robot.RobotMap;

public class NDiUnits {
    /* INCHS CONVERSION */

    public static double inchToFeet = 12;
    public static double inchToFeet(double inch) {
        return inch * inchToFeet;
    }

    public static double inchToEncoderTicks = 1f / RobotMap.driveTrainWheelCircumference * RobotMap.driveTrainTicksPerRevelution;
    public static double inchToEncoderTicks(double inch) {
        return inch * inchToEncoderTicks;
    }

    public static double inchToMeter = 0.0254f;
    public static double inchToMeter(double inch) {
        return inch * inchToMeter;
    }

    /* FEET CONVERSION */

    public static double feetToInch = 1f / 12f;
    public static double feetToInch(double feet) {
        return feet * feetToInch;
    }

    public static double feetToMeter = 0.3048f;
    public static double feetToMeter(double feet) {
        return feet * feetToMeter;
    }

    public static double feetToEncoderTicks = inchToEncoderTicks * inchToFeet;
    public static double feetToEncoderTicks(double feet) {
        return feet * feetToEncoderTicks;
    }

    /* METER CONVERSION */

    public static double meterToInch = 1f / inchToMeter;
    public static double meterToInch(double feet) {
        return feet * meterToInch;
    }

    public static double meterToFeet = 1f / feetToMeter;
    public static double meterToFeet(double feet) {
        return feet * meterToFeet;
    }

    public static double meterToEncoderTicks = inchToEncoderTicks * inchToMeter;
    public static double meterToEncoderTicks(double feet) {
        return feet * meterToEncoderTicks;
    }

    /* ENCODER TICKS CONVERSION */

    public static double encoderTicksToInch = 1f / inchToEncoderTicks;
    public static double encoderTicksToInch(double encoderTicks) {
        return encoderTicks * encoderTicks;
    }

    public static double encoderTicksToFeet = 1f / feetToEncoderTicks;
    public static double encoderTicksToFeet(double encoderTicks) {
        return encoderTicks * encoderTicks;
    }

    public static double encoderTicksToMeter = 1f / meterToEncoderTicks;
    public static double encoderTicksToMeter(double encoderTicks) {
        return encoderTicks * encoderTicks;
    }

    /* */

    public static double revsPerMinuteToTicksPerTenth(double revPerMinute) {
        return 0.0;
    }

    /**
     * Distance:
     * - Encoder Ticks
     * - inch
     * - meter
     * Distance / Time:
     * - seconds
     * - hundths
     */
}
