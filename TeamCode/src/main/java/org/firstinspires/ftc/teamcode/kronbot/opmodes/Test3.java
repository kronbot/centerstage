package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;

@TeleOp(name = "Test 3")
public class Test3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor motor = new Motor(hardwareMap);
        motor.init("motor", false, false, false, true, true, true);

        int position = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.circle) {
                position = 0;
                motor.setTargetPosition(position);
            } if (gamepad1.triangle) {
                position = 1000;
                motor.setTargetPosition(position);
            } if (gamepad1.square) {
                position = 2000;
                motor.setTargetPosition(position);
            } if (gamepad1.cross) {
                position = 3000;
                motor.setTargetPosition(position);
            }

            if (Math.abs(motor.getCurrentPosition() - position) < 50)
                motor.setPower(0);
            else if (motor.getCurrentPosition() < position) {
                motor.setPower(0.75);
                telemetry.addLine("Forward");
            } else {
                motor.setPower(-0.75);
                telemetry.addLine("Backward");
            }

            telemetry.addData("Wanted Position", position);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
