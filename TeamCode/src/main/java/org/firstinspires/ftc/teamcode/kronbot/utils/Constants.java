package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Methods.cmToInch;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Constants for KronBot viewable from dashboard
 *
 * @version 1.0
 */
@Config
public class Constants {
    public static final String testGroup = "test";
    public static final String mainGroup = "main";

    // Distance between the left and right deadwheels
    public static final double lateralError = 1;

    // Distance from the center of the robot to the deadwheels
    public static final Pose2d leftEncoderPos = new Pose2d(cmToInch(1), cmToInch(1), 0);
    public static final Pose2d rightEncoderPos = new Pose2d(cmToInch(1), cmToInch(1), 0);
    public static final Pose2d middleEncoderPos = new Pose2d(cmToInch(1), cmToInch(1), Math.toRadians(90));
}
