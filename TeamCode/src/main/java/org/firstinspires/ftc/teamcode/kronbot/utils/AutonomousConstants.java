package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

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

    public static Coordinates RedPixelLeft = new Coordinates(60, 15, 0);
    public static Coordinates RedPixelMiddle = new Coordinates(110, -15, 0);
    public static Coordinates RedPixelRight = new Coordinates(60, -55, 0);
    public static Coordinates middleOfTile = new Coordinates(60,0,Math.toRadians(90));

    public static Pose2d coordinatesConvert(Coordinates coord) {
        return new Pose2d(coord.x, coord.y, coord.heading);
    }
}