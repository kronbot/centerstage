package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

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

    public static Coordinates StartPoseLeftRed = new Coordinates(-35, -70 + 15/2, 270);
    public static Coordinates StartPoseRightRed = new Coordinates(12, -70 + 15/2, 270);
    public static Coordinates StartPoseLeftBlue = new Coordinates(-35, 70 - 15/2, 90);
    public static Coordinates StartPoseRightBlue = new Coordinates(12, 70 - 15/2, 90);

    public static Coordinates PixelLeft = new Coordinates(-4, 31, 130);
    public static Coordinates PixelMiddle = new Coordinates(0, 30, 90);
    public static Coordinates PixelRight = new Coordinates(4, 31, 40);
    public static Coordinates PixelForward = new Coordinates(0, 10, 0);

    public static Coordinates Back = new Coordinates(0,-40,0);
    public static Coordinates FarBack = new Coordinates(0, -59, 270);

    public static Coordinates ClosePark = new Coordinates(45, -35, 180);

    public static Pose2d coordinatesConvert(Coordinates coord) {
        return new Pose2d(coord.x, coord.y, Math.toRadians(coord.heading));
    }
}
