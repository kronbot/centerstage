package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.checkerframework.checker.units.qual.C;

@Config
public final class AutonomousConstants {

    public static class Coordinates {
        public double y;
        public double x;
        public double heading;

        public Coordinates(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static Coordinates RedPixelLeft = new Coordinates(25, 5, 0);
    public static Coordinates RedPixelMiddle = new Coordinates(38.5, -5, 0);
    public static Coordinates RedPixelRight = new Coordinates(25, -15, 0);

    public static Coordinates Parking = new Coordinates(25, 40, 0);

    public static Pose2d coordinatesConvert(Coordinates coord) {
        return new Pose2d(coord.x, coord.y, coord.heading);
    }
}
