package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

@TeleOp(name = "Test", group = Constants.MAIN_GROUP)
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        Button servoButton = new Button();
        Servo servo = new Servo(hardwareMap);
        servo.init("servo", false, false);

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;

            servoButton.updateButton(gamepad1.circle);
            servoButton.shortPress();

            telemetry.addData("(X;Y;R): ",
                    x + " " + y + " " + r);
            telemetry.update();

            if (servoButton.getShortToggle()) {
                servo.setPosition(0.5);
            } else {
                servo.setPosition(0.23);
            }

            if (x > 0.25 && y > 0.25 || x < -0.25 && y < -0.25) {
                leftFront.setPower(y);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(y);
            } else if (x < -0.25 && y > 0.25 || x > 0.25 && y < -0.25) {
                leftFront.setPower(0);
                leftRear.setPower(y);
                rightFront.setPower(y);
                rightRear.setPower(0);
            } else if (y > 0.25 || y < -0.25) {
                leftFront.setPower(y);
                leftRear.setPower(y);
                rightFront.setPower(y);
                rightRear.setPower(y);
            } else if (x > 0.25 || x < -0.25) {
                leftFront.setPower(x);
                leftRear.setPower(-x);
                rightFront.setPower(-x);
                rightRear.setPower(x);
            } else if (r > 0.25 || r < -0.25) {
                leftFront.setPower(r);
                leftRear.setPower(r);
                rightFront.setPower(-r);
                rightRear.setPower(-r);
            } else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
        }
    }
}
