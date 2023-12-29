package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HOOK_POWER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Handles the hang motors.
 * It initializes the motors given a set of parameters and then it is ready to use
 *
 * @version 1.0
 */
public class HangDriver {
    public DcMotorEx motor;
    public DcMotorEx motor2;

    public void init(DcMotorEx motor, DcMotorEx motor2) {
        this.motor = motor;
        this.motor2 = motor2;

        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(boolean forwardButton, boolean reverseButton) {
        if (forwardButton) {
            motor.setPower(-HOOK_POWER);
            motor2.setPower(-HOOK_POWER);
        } else if (reverseButton) {
            motor.setPower(HOOK_POWER);
            motor2.setPower(HOOK_POWER);
        } else {
            motor.setPower(0);
            motor2.setPower(0);
        }
    }
}
