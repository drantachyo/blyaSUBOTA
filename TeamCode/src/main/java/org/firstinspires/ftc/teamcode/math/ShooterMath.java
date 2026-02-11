package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.Range;

public class ShooterMath {

    public static double MIN_DIST = 29.7;
    public static double MAX_DIST = 72.2;

    public static double calculateRPM(double distance) {
        double x = Range.clip(distance, MIN_DIST, MAX_DIST);

        double y = 0.00571245 * Math.pow(x, 3)
                - 0.711202   * Math.pow(x, 2)
                + 43.02844   * x
                + 2480.40674;
        return Range.clip(y, 0, 4800);
    }
    public static double calculateHood(double distance) {
        double x = Range.clip(distance, MIN_DIST, MAX_DIST);

        double y = -0.00000238679 * Math.pow(x, 3)
                + 0.000803582    * Math.pow(x, 2)
                - 0.0718718      * x
                + 1.95;
        return Range.clip(y, 0.0, 1.0);
    }
}