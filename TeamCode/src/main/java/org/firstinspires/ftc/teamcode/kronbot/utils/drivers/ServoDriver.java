package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM1_INIT_POS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM2_INIT_POS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM1_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM2_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;

import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class ServoDriver {
    Servo pixelServo;
    Servo armServo1;
    Servo armServo2;
    Servo hookServo1;
    Servo hookServo2;
    Servo intakeServo;

    public void init(Servo armServo1, Servo armServo2,  Servo pixelServo, Servo hookServo1, Servo hookServo2,Servo intakeServo) {
        armServo1.setPWMRange(500, 2500);
        armServo2.setPWMRange(500, 2500);
        hookServo1.setPWMRange(500, 2500);
        pixelServo.setPWMRange(500, 2500);
        hookServo2.setPWMRange(500,2500);
        intakeServo.setPWMRange(500,2500);

        this.pixelServo = pixelServo;
        this.armServo1 = armServo1;
        this.armServo2 = armServo2;
        this.hookServo1 = hookServo1;
        this.hookServo2 = hookServo2;
        this.intakeServo = intakeServo;
    }

    public void arm(boolean activated) {
        if (activated) {
            armServo1.setPosition(ARM1_POSITION);
            armServo2.setPosition(ARM2_POSITION);
        } else {
            armServo1.setPosition(ARM1_INIT_POS);
            armServo2.setPosition(ARM2_INIT_POS);
        }
    }

    public void pixel(boolean activated) {
        if (activated) pixelServo.setPosition(1);
        else pixelServo.setPosition(0);
    }

    public void hook(boolean activated) {
        if (activated) {
            hookServo1.setPosition(0);
            hookServo2.setPosition(0);
        } else {
            hookServo1.setPosition(0.7);
            hookServo2.setPosition(0.7);
        }
    }

    public void intake(boolean activated) {
        if(activated) intakeServo.setPosition(1);
        else intakeServo.setPosition(0.5);
    }

    public void cutArmPower() {
        armServo1.cutPower();
        armServo2.cutPower();
    }

    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value;
    }
}
