package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SPEED;

import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class ServoDriver {
    Servo intakeServo;
    Servo armServo1;
    Servo armServo2;
    Servo clawServo;

    public void init(Servo armServo1, Servo armServo2,  Servo intakeServo, Servo clawServo) {
        armServo1.setPWMRange(500, 2500);
        armServo2.setPWMRange(500, 2500);
        clawServo.setPWMRange(500, 2500);
        intakeServo.setPWMRange(500, 2500);

        this.intakeServo = intakeServo;
        this.armServo1 = armServo1;
        this.armServo2 = armServo2;
        this.clawServo = clawServo;
    }

    public void intake(boolean action) {
        if (action)
            intakeServo.setPosition(0.7);
        else intakeServo.setPosition(0);
    }

    public void claw(double position) {
        position = addons(position);
        if (position == 0) return;
        if (position > 0 && clawServo.getPosition() < 0.99 || position < 0 && clawServo.getPosition() > 0.01)
            clawServo.setPosition(clawServo.getPosition() + 0.001 * position);
    }
    

    public void arm(double position) {
        position = addons(position);
        if (position == 0) return;
        if (position > 0 && armServo1.getPosition() < 0.5 || position < 0 && armServo1.getPosition() > 0.01)
            armServo1.setPosition(armServo1.getPosition() + 0.0025 * position);
        if (position > 0 && armServo2.getPosition() > 0.5 || position < 0 && armServo2.getPosition() < 0.99)
            armServo2.setPosition(armServo2.getPosition() - 0.0025 * position);
    }

    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value;
    }
}
