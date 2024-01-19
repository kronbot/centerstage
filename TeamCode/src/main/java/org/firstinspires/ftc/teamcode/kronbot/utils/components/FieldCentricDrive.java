package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ROBOT_SPEED;

/**
 * Field centric drive is a drive system that allows the robot to move in a direction relative to the field
 *
 * @version 1.0
 */
@Config
public class FieldCentricDrive {
    KronBot robot;
    Gamepad gamepad;

    double rotatedX = 0;
    double rotatedY = 0;


    public FieldCentricDrive(KronBot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void run() {
        robot.gyroscope.updateOrientation();
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        double neededOffset = -Math.toRadians(robot.gyroscope.getHeading());

        rotatedX = x * Math.cos(neededOffset) - y * Math.sin(neededOffset);
        rotatedY = x * Math.sin(neededOffset) + y * Math.cos(neededOffset);

        rotatedX = addons(rotatedX);
        rotatedY = addons(rotatedY);
        r = addons(r);

        double normalizer = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(r), 1.0);

        double leftFrontPower = (rotatedY + rotatedX + r) / normalizer;
        double leftRearPower = (rotatedY - rotatedX + r) / normalizer;
        double rightRearPower = (rotatedY + rotatedX - r) / normalizer;
        double rightFrontPower = (rotatedY - rotatedX - r) / normalizer;

        robot.motors.leftFront.setPower(leftFrontPower);
        robot.motors.leftRear.setPower(leftRearPower);
        robot.motors.rightRear.setPower(rightRearPower);
        robot.motors.rightFront.setPower(rightFrontPower);
    }

    private double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * ROBOT_SPEED;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("---FIELD CENTRIC DRIVE---");

        telemetry.addData("Speed Multiplier", ROBOT_SPEED);
        telemetry.addData("Robot Angle", robot.gyroscope.getHeading());

        telemetry.addData("rotatedX", rotatedX);
        telemetry.addData("rotatedY", rotatedY);

        telemetry.addData("LeftRear Position", robot.motors.leftRear.getCurrentPosition());
        telemetry.addData("RightRear Position", robot.motors.rightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position", robot.motors.leftFront.getCurrentPosition());
        telemetry.addData("RightFront Position", robot.motors.rightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power", robot.motors.leftRear.getPower());
        telemetry.addData("RightRear Power", robot.motors.rightRear.getPower());
        telemetry.addData("LeftFront Power", robot.motors.leftFront.getPower());
        telemetry.addData("RightFront Power", robot.motors.rightFront.getPower());
    }
}
