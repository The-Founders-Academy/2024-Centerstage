package org.firstinspires.ftc.teamcode.utility;

public class MathUtil {
    public static double clamp(double value, double min, double max) {
        double returnValue = value;
        if(value > max) {
            returnValue = max;
        } else if(value < min) {
            returnValue = min;
        }
        return returnValue;
    }
}
