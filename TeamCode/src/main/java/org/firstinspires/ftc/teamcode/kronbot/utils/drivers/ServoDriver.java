package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM1_HIGH;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM1_INIT_POS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM2_HIGH;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM2_INIT_POS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM1_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM2_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HOOK1_INIT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HOOK2_INIT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.PLANE_END;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.PLANE_START;

import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class ServoDriver {
    Servo pixelServo;
    Servo pixelServo2;
    Servo armServo1;
    Servo armServo2;
    Servo hookServo1;
    Servo hookServo2;
    Servo intakeServo;
    Servo planeServo;

    public void init(Servo armServo1, Servo armServo2,  Servo pixelServo, Servo hookServo1, Servo hookServo2, Servo intakeServo, Servo planeServo, Servo pixelServo2) {
        armServo1.setPWMRange(500, 2500);
        armServo2.setPWMRange(500, 2500);
        hookServo1.setPWMRange(500, 2500);
        pixelServo.setPWMRange(500, 2500);
        hookServo2.setPWMRange(500,2500);
        intakeServo.setPWMRange(500,2500);
        planeServo.setPWMRange(500, 2500);
        pixelServo2.setPWMRange(500, 2500);

        this.pixelServo = pixelServo;
        this.armServo1 = armServo1;
        this.armServo2 = armServo2;
        this.hookServo1 = hookServo1;
        this.hookServo2 = hookServo2;
        this.intakeServo = intakeServo;
        this.planeServo = planeServo;
        this.pixelServo2 = pixelServo2;
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

    public void high(boolean activated) {
        if (activated) {
            armServo1.setPosition(ARM1_HIGH);
            armServo2.setPosition(ARM2_HIGH);
        }
    }

    public void pixel(boolean activated) {
        if (activated) {
            pixelServo.setPosition(1);
            pixelServo2.setPosition(0);
        }
        else {
            pixelServo.setPosition(0);
            pixelServo2.setPosition(0.5);
        }
    }

    public void hook(boolean activated) {
        if (activated) {
            hookServo1.setPosition(0.35);
            hookServo2.setPosition(0.45);
        } else {
            hookServo1.setPosition(HOOK1_INIT);
            hookServo2.setPosition(HOOK2_INIT);
        }
    }

    public void intakeSpinUp(boolean activated) {
        if(activated) intakeServo.setPosition(1);
        else intakeServo.setPosition(0.5);
    }
    public void intakeSpinDown(boolean activated) {
        if(activated) intakeServo.setPosition(0);
        else intakeServo.setPosition(0.5);
    }

    public void plane(boolean activated) {
        if (activated) planeServo.setPosition(PLANE_END);
        else planeServo.setPosition(PLANE_START);
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
