package org.firstinspires.ftc.teamcode.kronbot.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Gyroscope;

/**
 * Field centric drive is a drive system that allows the robot to move in a direction relative to the field
 *
 * @version 1.0
 */
@Config
public class FieldCentricDrive {
    MotorDriver motors;
    Gamepad gamepad;

    Gyroscope gyroscope;

    double speed = 1.0;
    public static final double controllerDeadzone = 0.15;

    double rotatedX = 0;
    double rotatedY = 0;

    public FieldCentricDrive(MotorDriver motors, Gamepad gamepad, Gyroscope gyroscope) {
        this.motors = motors;
        this.gamepad = gamepad;
        this.gyroscope = gyroscope;
    }

    public void run() {
        gyroscope.updateOrientation();
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        double neededOffset = -Math.toRadians(gyroscope.getHeading());

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

        motors.leftFront.setPower(leftFrontPower);
        motors.leftRear.setPower(leftRearPower);
        motors.rightRear.setPower(rightRearPower);
        motors.rightFront.setPower(rightFrontPower);
    }

    private double addons(double value) {
        if (Math.abs(value) < controllerDeadzone) return 0;
        return value * speed;
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addLine("---FIELD CENTRIC DRIVE---");

        telemetry.addData("Speed Multiplier", speed);
        telemetry.addData("Robot Angle", gyroscope.getHeading());

        telemetry.addData("rotatedX", rotatedX);
        telemetry.addData("rotatedY", rotatedY);

        telemetry.addData("LeftRear Position", motors.leftRear.getCurrentPosition());
        telemetry.addData("RightRear Position", motors.rightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position", motors.leftFront.getCurrentPosition());
        telemetry.addData("RightFront Position", motors.rightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power", motors.leftRear.getPower());
        telemetry.addData("RightRear Power", motors.rightRear.getPower());
        telemetry.addData("LeftFront Power", motors.leftFront.getPower());
        telemetry.addData("RightFront Power", motors.rightFront.getPower());
    }
}
