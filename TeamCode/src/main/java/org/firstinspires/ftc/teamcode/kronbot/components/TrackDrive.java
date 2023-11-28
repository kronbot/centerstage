package org.firstinspires.ftc.teamcode.kronbot.components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;

public class TrackDrive {
    KronBot robot;
    Gamepad gamepad;
    double reverse;

    public TrackDrive(KronBot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void run() {
        double x = gamepad.left_stick_x;
        double r = -gamepad.right_stick_y;

        robot.motors.leftFront.setPower(x + r);
        robot.motors.leftRear.setPower(x + r);
        robot.motors.rightFront.setPower(x - r);
        robot.motors.rightRear.setPower(x - r);
    }

    public void setReverse(boolean isReverse) {
        if (isReverse) reverse = -1.0;
        else reverse = 1.0;
    }
}
