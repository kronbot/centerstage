package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Handles all of the 4 driving motors.
 * It initializes all of the motors given a set of parameters and then they are ready to use
 *
 * @version 1.0
 */
public class MotorDriver {
    public DcMotorEx leftRear, leftFront, rightRear, rightFront;
    HardwareMap hardwareMap;

    public MotorDriver(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void Init(DcMotorEx leftRear, DcMotorEx leftFront, DcMotorEx rightRear, DcMotorEx rightFront) {
        this.leftRear = leftRear;
        this.leftFront = leftFront;
        this.rightRear = rightRear;
        this.rightFront = rightFront;

        ArrayList<DcMotorEx> motors = new ArrayList<DcMotorEx>(Arrays.asList(leftRear, rightRear, leftFront, rightFront));

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
    }
}
