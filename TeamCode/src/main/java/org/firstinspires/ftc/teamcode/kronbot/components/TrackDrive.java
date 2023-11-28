package org.firstinspires.ftc.teamcode.kronbot.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

@Config
public class TrackDrive {
    KronBot robot;
    Gamepad gamepad;
    double reverse;
    Servo intakeServo;
    Servo armServo;
    HardwareMap hardwareMap;

    public static Integer power1=500;
    public static Integer power2=2500;
    public static double closed1=0;
    public static double open1=0.7;
    public TrackDrive(KronBot robot, Gamepad gamepad,HardwareMap hardwareMap) {
        this.robot = robot;
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        intakeServo = new Servo(hardwareMap);
        intakeServo.init("intake", false, true);
        intakeServo.setPWMRange(500, 2500);
        armServo = new Servo(hardwareMap);
        armServo.init("arm", false, true);
        armServo.setPWMRange(power1, power2);
    }
    public enum ClawModes
    {
        CLOSED(0),
        OPEN(0.7),
        CLOSED1(closed1),
        OPEN1(open1);

        public double position = 0;

        ClawModes(double value) {
            this.position = value;
        }
    }
    ClawModes intakeState = ClawModes.OPEN;
    Button intakeButton = new Button();
    ClawModes armState = ClawModes.OPEN1;
    Button armButton = new Button();
    public void run() {
        robot.motors.leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        robot.motors.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        double y = -gamepad.left_stick_y;
        double r = -gamepad.right_stick_y;

        intakeButton.updateButton(gamepad.dpad_right);
        intakeButton.toggle();

        if (intakeButton.getToggleStatus())
            intakeState = ClawModes.CLOSED;
        else
            intakeState = ClawModes.OPEN;

        intakeServo.setPosition(intakeState.position);

        armButton.updateButton(gamepad.dpad_up);
        armButton.toggle();

        if (armButton.getToggleStatus())
            armState = ClawModes.CLOSED1;
        else
            armState = ClawModes.OPEN1;

        armServo.setPosition(armState.position);

        robot.motors.leftFront.setPower(y + r);
        robot.motors.leftRear.setPower(y + r);
        robot.motors.rightFront.setPower(y - r);
        robot.motors.rightRear.setPower(y - r);
    }

    public void setReverse(boolean isReverse) {
        if (isReverse) reverse = -1.0;
        else reverse = 1.0;
    }
}
