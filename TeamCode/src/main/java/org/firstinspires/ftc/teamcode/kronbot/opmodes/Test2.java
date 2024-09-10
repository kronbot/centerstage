package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

@TeleOp(name = "Test2", group = Constants.MAIN_GROUP)
public class Test2 extends LinearOpMode {
    KronBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initIMU(hardwareMap);

        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            robot.gyroscope.updateOrientation();
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;

            double angle = -Math.toRadians(robot.gyroscope.getHeading());

            double xRotated = x * Math.cos(angle) - y * Math.sin(angle);
            double yRotated = y * Math.sin(angle) + y * Math.cos(angle);

            xRotated = joystick(xRotated);
            yRotated = joystick(yRotated);
            r = joystick(r);

            double normalize = Math.max(Math.abs(yRotated) + Math.abs(xRotated) + Math.abs(r), 1);
            double leftFrontPower = (yRotated + xRotated + r) / normalize;
            double leftRearPower = (yRotated - xRotated + r) / normalize;
            double rightFrontPower = (yRotated - xRotated - r) / normalize;
            double rightRearPower = (yRotated + xRotated - r) / normalize;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
        }
    }
    double joystick (double x) {
        return x>0.1 || x<-0.1 ? x:0;
    }
}
