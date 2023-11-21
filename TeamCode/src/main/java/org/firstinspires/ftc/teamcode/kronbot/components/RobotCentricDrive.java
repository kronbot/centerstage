package org.firstinspires.ftc.teamcode.kronbot.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SPEED;

/**
 * Robot centric drive is a drive system that allows the robot to move in a direction relative to the robot
 *
 * @version 1.0
 */
@Config
public class RobotCentricDrive {
    KronBot robot;
    Gamepad gamepad;

    double reverse = 1.0;

    public RobotCentricDrive(KronBot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void run() {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        x = addons(x) * reverse;
        y = addons(y) * reverse;
        r = addons(r);

        double normalizer = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1.0);

        double leftFrontPower = (y + x + r) / normalizer;
        double rightFrontPower = (y - x - r) / normalizer;
        double leftRearPower = (y - x + r) / normalizer;
        double rightRearPower = (y + x - r) / normalizer;

        robot.motors.leftFront.setPower(leftFrontPower);
        robot.motors.rightFront.setPower(rightFrontPower);
        robot.motors.leftRear.setPower(leftRearPower);
        robot.motors.rightRear.setPower(rightRearPower);
    }

    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * SPEED;
    }

    public void setReverse(boolean isReverse) {
        if (isReverse) reverse = -1.0;
        else reverse = 1.0;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("---ROBOT CENTRIC DRIVE---");

        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", SPEED);

        telemetry.addData("LeftRear Position: ", robot.motors.leftRear.getCurrentPosition());
        telemetry.addData("RightRear Position: ", robot.motors.rightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position: ", robot.motors.leftFront.getCurrentPosition());
        telemetry.addData("RightFront Position: ", robot.motors.rightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power: ", robot.motors.leftRear.getPower());
        telemetry.addData("RightRear Power: ", robot.motors.rightRear.getPower());
        telemetry.addData("LeftFront Power: ", robot.motors.leftFront.getPower());
        telemetry.addData("RightFront Power: ", robot.motors.rightFront.getPower());
    }
}
