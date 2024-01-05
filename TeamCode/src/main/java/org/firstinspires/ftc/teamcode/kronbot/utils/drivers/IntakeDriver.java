package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_POWER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Handles the intake motor.
 * It initializes the motor given a set of parameters and then it is ready to use
 *
 * @version 1.0
 */
public class IntakeDriver {
    public DcMotorEx motor;

    public void init(DcMotorEx motor) {
        this.motor = motor;

        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(boolean forwardButton, boolean reverseButton) {
        if (forwardButton) {
            motor.setPower(-INTAKE_POWER);
        }
        else if (reverseButton)
            motor.setPower(INTAKE_POWER);
        else
            motor.setPower(0);

    }
}
