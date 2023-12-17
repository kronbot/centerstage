package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_FIRST_OPEN_POS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_SECOND_OPEN_POS;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class ServoDriver {
    Servo intakeServo;
    Servo armServo1;
    Servo armServo2;
    Servo clawServo;
    Servo planeServo;

    public void init(Servo armServo1, Servo armServo2,  Servo intakeServo, Servo clawServo, Servo planeServo) {
        armServo1.setPWMRange(500, 2500);
        armServo2.setPWMRange(500, 2500);
        clawServo.setPWMRange(500, 2500);
        intakeServo.setPWMRange(500, 2500);
        planeServo.setPWMRange(500,2500);

        this.intakeServo = intakeServo;
        this.armServo1 = armServo1;
        this.armServo2 = armServo2;
        this.clawServo = clawServo;
        this.planeServo = planeServo;
    }

//    public void intakeOpen() {
//        if (intakeServo.getPosition() == INTAKE_FIRST_OPEN_POS)
//            intakeServo.setPosition(INTAKE_SECOND_OPEN_POS);
//        else if (intakeServo.getPosition() == INTAKE_CLOSED_POS)
//            intakeServo.setPosition(INTAKE_FIRST_OPEN_POS);
//    }

    public void intakeClose() {
        intakeServo.setPosition(INTAKE_CLOSED_POS);
    }
    public void intakeOpen() {
        intakeServo.setPosition(INTAKE_FIRST_OPEN_POS);
    }

    public void intake(boolean position) {
        if (position) intakeServo.setPosition(INTAKE_SECOND_OPEN_POS);
        else intakeServo.setPosition(INTAKE_CLOSED_POS);
    }

//    public void plane(boolean position) {
//        if (position)
//            intakeServo.setPosition(0);
//        else
//            intakeServo.setPosition(1);
//    }

    public void claw(double position) {
        position = addons(position);
        if (position == 0) return;
        if (position > 0 && clawServo.getPosition() < 0.99 || position < 0 && clawServo.getPosition() > 0.01)
            clawServo.setPosition(clawServo.getPosition() + 0.001 * position);
    }


    public void arm(double position) {
        position = addons(position);
        if (position == 0) return;
        if (position > 0 && armServo1.getPosition() < 0.99 || position < 0 && armServo1.getPosition() > 0.1)
            armServo1.setPosition(armServo1.getPosition() + Constants.SERVO_SPEED * position);
        if (position > 0 && armServo2.getPosition() > 0.15 || position < 0 && armServo2.getPosition() < 0.9)
            armServo2.setPosition(armServo2.getPosition() - Constants.SERVO_SPEED * position);
    }

    private double getArmAngle() {
        return -300 * armServo1.getPosition() + 358.5;
    }

    private double getClawAngle() {
        return 300 * clawServo.getPosition() - 78;
    }

    private double getClawPosition(double angle) {
        return (angle + 78) / 300;
    }

    private double getPerpendicularClawPosition() {
        double base = Constants.ARM1_INIT_POS;
        double transformed = Constants.CLAW_INIT_POS;
        double steps = (base - armServo1.getPosition()) / 0.05;

        return transformed - steps * Constants.INTAKE_STEPS;
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
