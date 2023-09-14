package org.firstinspires.ftc.teamcode.kronbot.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.MotorDriver;

/**
 * Robot centric drive is a drive system that allows the robot to move in a direction relative to the robot
 *
 * @version 1.0
 */
@Config
public class RobotCentricDrive {
    MotorDriver motors;
    Gamepad gamepad;

    double speed = 1.0;

    public static double controllerDeadzone = 0.15;
    double reverse = 1.0;

    public RobotCentricDrive(MotorDriver motors, Gamepad gamepad) {
        this.motors = motors;
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

        motors.leftFront.setPower(leftFrontPower);
        motors.rightFront.setPower(rightFrontPower);
        motors.leftRear.setPower(leftRearPower);
        motors.rightRear.setPower(rightRearPower);
    }

    public double addons(double value) {
        if (Math.abs(value) < controllerDeadzone) return 0;
        return value * speed;
    }

    public void setReverse(boolean isReverse) {
        if (isReverse) reverse = -1.0;
        else reverse = 1.0;
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addLine("---ROBOT CENTRIC DRIVE---");

        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", speed);

        telemetry.addData("LeftRear Position: ", motors.leftRear.getCurrentPosition());
        telemetry.addData("RightRear Position: ", motors.rightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position: ", motors.leftFront.getCurrentPosition());
        telemetry.addData("RightFront Position: ", motors.rightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power: ", motors.leftRear.getPower());
        telemetry.addData("RightRear Power: ", motors.rightRear.getPower());
        telemetry.addData("LeftFront Power: ", motors.leftFront.getPower());
        telemetry.addData("RightFront Power: ", motors.rightFront.getPower());
    }
}
