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
    public static double CLAW_INIT_POS = 0.26;

    public static double INTAKE_CLOSED_POS = 0;
    public static double INTAKE_FIRST_OPEN_POS = 0.6;
    public static double INTAKE_SECOND_OPEN_POS = 1;
    public static double INTAKE_STEPS = 0.035;

    public static double ARM_TRASHHOLD_90_DEGREES = 0.7;
}
