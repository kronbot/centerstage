package org.firstinspires.ftc.teamcode.kronbot.utils.pid;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Various math utilities.
 */
public class MathUtil {

    private static final double EPSILON = 1e-6;

    /**
     * Returns the real solutions to the quadratic ax^2 + bx + c.
     */
    public static List<Double> solveQuadratic(double a, double b, double c) {
        double disc = b * b - 4 * a * c;
        if (epsilonEquals(disc, 0.0)) {
            return Collections.singletonList(-b / (2 * a));
        } else if (disc > 0.0) {
            List<Double> solutions = new ArrayList<>();
            solutions.add((-b + Math.sqrt(disc)) / (2 * a));
            solutions.add((-b - Math.sqrt(disc)) / (2 * a));
            return solutions;
        } else {
            return Collections.emptyList();
        }
    }

    /**
     * Checks if two doubles are "close enough" (epsilon equals).
     */
    public static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }
}
