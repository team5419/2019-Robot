package frc.robot.util;

import frc.robot.RobotMap;

public class NDiMath {
    public static double encoderTicksToFeet(double encoderTicks) {
        return encoderTicks * RobotMap.driveTrainWheelCircumference / RobotMap.driveTrainTicksPerRevelution;
    }

    public static double feetToEncoderTicks(double feet) {
        return feet / RobotMap.driveTrainWheelCircumference * RobotMap.driveTrainTicksPerRevelution;
    }

    public static double revsPerMinuteToTicksPerTenth(double revPerMinute) {
        return 0.0;
    }
}
