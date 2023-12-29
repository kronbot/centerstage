package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for KronBot viewable from dashboard
 *
 * @version 1.0
 */
@Config
public class Constants {
    public final static String TEST_GROUP = "test";
    public final static String MAIN_GROUP = "main";

    public static double CONTROLLER_DEADZONE = 0.15;

    public static double ROBOT_SPEED = 1.0;
    public static double SLIDES_SPEED = 1.0;
    public static double REST_POWER = 0.03;

    public static double SERVO_SPEED = 0.001;

    public static double ARM1_INIT_POS = 0.7;
    public static double ARM2_INIT_POS = 0.3;

    public static double INTAKE_POWER = 1.0;
    public static double HOOK_POWER = 1.0;

    public static double LIFT_TOLERANCE = 50;
    public static int LIFT_INIT_POSITION = 1000;
    public static int LIFT_MAX_POSITION = 2600;
    public static double LIFT_REVERSE_CONSTANT = 0.75;
}
