package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SPEED;

import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class ServoDriver {
    Servo intakeServo;
    Servo armServo;
    Servo clawServo;

    public static double closed1=0;
    public static double open1=0.7;

    public void init(Servo armServo, Servo intakeServo, Servo clawServo) {
        armServo.setPWMRange(500, 2500);
        clawServo.setPWMRange(500, 2500);
        intakeServo.setPWMRange(500, 2500);
    }

    public void intake(boolean action) {
        if (action)
            clawServo.setPosition(1);
        else clawServo.setPosition(0);
    }

    public void claw(double position) {
        position = addons(position);
        clawServo.setPosition(clawServo.getPosition() + position * 2);
    }

    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * SPEED;
    }
}
