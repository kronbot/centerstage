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
    public static double REST_POWER = 0.05;

    public static double INTAKE_POWER = 1.0;
    public static double HOOK_POWER = 1.0;

    public static double LIFT_TOLERANCE = 50;
    public static int   LIFT_INIT_POSITION = 100 ;
    public static int LIFT_MAX_POSITION = 5000;

    public static double LIFT_REVERSE_CONSTANT = 0.75;

    public static double ARM1_INIT_POS = 0.35;
    public static double ARM2_INIT_POS = 0.30;
    public static double ARM1_POSITION = 0.72;
    public static double ARM2_POSITION = 0.623;
    public static double ARM1_HIGH = 0.00;
    public static double ARM2_HIGH = 0.23;
    public static double HOOK1_INIT = 0.23;
    public static double HOOK2_INIT = 0.89;
    public static double HOOK1_POS = 0.515;
    public static double HOOK2_POS = 0.64;
    public static double HOOK1_2POS = 0.25;
    public static double HOOK2_2POS = 0.4;
    public static double CAMERA_TRASH_HOLD = 0.2;

    public static double PLANE_START = 0.7;
    public static double PLANE_END = 1;

    public static int BUTTON_LONG_PRESS_TIME = 750;

    public static int BLUE_HUE_LOW = 0;
    public static int BLUE_HUE_HIGH = 180;
}
