package org.firstinspires.ftc.teamcode.util;

public class MathUtil {

    public static double clamp(double number, double lowerBound, double upperBound) {
        if (number < lowerBound) {
            return lowerBound;
        } else if (number > upperBound) {
            return upperBound;
        }

        return number;
    }

}
