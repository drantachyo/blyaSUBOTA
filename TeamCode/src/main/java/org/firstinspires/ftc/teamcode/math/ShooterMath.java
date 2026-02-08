package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.Range;

public class ShooterMath {

    // === BOUNDARIES ===
    // Updated based on the data tables in the images (29.7 to 72.2)
    public static double MIN_DIST = 29.7;
    public static double MAX_DIST = 72.2;

    /**
     * Calculates RPM based on cubic regression from Image 2.
     * Equation: y = 0.00571245x^3 - 0.711202x^2 + 43.02844x + 2469.40674
     */
    public static double calculateRPM(double distance) {
        // Clip distance to stay within valid data range
        double x = Range.clip(distance, MIN_DIST, MAX_DIST);

        double y = 0.00571245 * Math.pow(x, 3)
                - 0.711202   * Math.pow(x, 2)
                + 43.02844   * x
                + 2480.40674;

        // Clip result to ensure safety (adjust max RPM if needed)
        return Range.clip(y, 0, 4800);
    }

    /**
     * Calculates Hood position based on cubic regression from Image 1.
     * Equation: y = -0.00000238679x^3 + 0.000803582x^2 - 0.0718718x + 1.95
     */
    public static double calculateHood(double distance) {
        // Clip distance to stay within valid data range
        double x = Range.clip(distance, MIN_DIST, MAX_DIST);

        double y = -0.00000238679 * Math.pow(x, 3)
                + 0.000803582    * Math.pow(x, 2)
                - 0.0718718      * x
                + 1.95;

        // Clip result to servo range (0.0 to 1.0)
        return Range.clip(y, 0.0, 1.0);
    }
}