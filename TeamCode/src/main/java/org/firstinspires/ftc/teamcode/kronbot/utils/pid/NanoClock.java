package org.firstinspires.ftc.teamcode.kronbot.utils.pid;

/**
 * Clock interface with nanosecond precision and no guarantee about its origin (that is, this is only suited for
 * measuring relative/elapsed time).
 */
public abstract class NanoClock {

    /**
     * Returns a NanoClock backed by System.nanoTime.
     */
    public static NanoClock system() {
        return new NanoClock() {
            @Override
            public double seconds() {
                return System.nanoTime() / 1e9;
            }
        };
    }

    /**
     * Returns the number of seconds since an arbitrary (yet consistent) origin.
     */
    public abstract double seconds();
}
