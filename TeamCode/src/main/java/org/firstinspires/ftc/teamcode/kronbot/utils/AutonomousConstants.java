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

    public static Coordinates PixelLeft = new Coordinates(30, 2, 50);
    public static Coordinates PixelMiddle = new Coordinates(30, 0, 0);
    public static Coordinates PixelRight = new Coordinates(30, -2, -50);
    public static Coordinates PixelForward = new Coordinates(12, 0, 0);

    public static Coordinates Back = new Coordinates(25,0,-90);

    public static Coordinates ClosePark = new Coordinates(25, -60, -90);
    public static Coordinates FarPark = new Coordinates(25, -120, -90);

    public static Pose2d coordinatesConvert(Coordinates coord) {
        return new Pose2d(coord.x, coord.y, Math.toRadians(coord.heading));
    }

    public static double MOTOR_POWER = 0.2;
    public static double MOTOR_SPIN_TIME = 1;
}
