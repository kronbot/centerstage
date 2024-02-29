package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ROBOT_SPEED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;

/**
 * Track drive is a drive system that allows the robot to move using tracks
 *
 * @version 1.0
 */
@Config
public class TrackDrive {
    KronBot robot;
    Gamepad gamepad;
    double reverse;

    public TrackDrive(KronBot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;

        robot.motors.leftRear.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void run() {
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        y = addons(y) * reverse;
        r = addons(r);

        double normalizer = Math.max(Math.abs(y) + Math.abs(r), 1.0);

        robot.motors.leftFront.setPower((y + r) / normalizer);
        robot.motors.leftRear.setPower((y + r) / normalizer);
        robot.motors.rightFront.setPower((y - r) / normalizer);
        robot.motors.rightRear.setPower((y - r) / normalizer);
    }

    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * ROBOT_SPEED;
    }

    public void setReverse(boolean isReverse) {
        if (isReverse) reverse = -1.0;
        else reverse = 1.0;
    }
}
