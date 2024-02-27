package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.HangDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.IntakeDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.LiftDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

public class KronBot {
    public MotorDriver motors;
    public IntakeDriver intake;
    public HangDriver hook;
    public LiftDriver lift;
    public Servo intakeServo;
    public Servo hookServoLeft, hookServoRight;
    public Servo armServoLeft, armServoRight, armServo;
    public Servo planeServo;
    public Servo clawServo;
    public ControlHubGyroscope gyroscope;

    public void initMotors(HardwareMap hardwareMap) {
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);
    }

    public void initLift(HardwareMap hardwareMap) {
        Motor liftMotor = new Motor(hardwareMap);
        lift = new LiftDriver();
        lift.init(liftMotor, false);
    }

    public void initHooks(HardwareMap hardwareMap) {
        DcMotorEx hangMotor = hardwareMap.get(DcMotorEx.class, "hangLeftMotor");
        DcMotorEx hangMotor2 = hardwareMap.get(DcMotorEx.class, "hangRightMotor");
        hook = new HangDriver();
        hook.init(hangMotor, hangMotor2);
        hookServoLeft = new Servo(hardwareMap);
        hookServoLeft.init("hookLeftServo", false, false, Constants.HOOK1_INIT, Constants.HOOK1_POS, Constants.HOOK1_2POS);
        hookServoRight = new Servo(hardwareMap);
        hookServoRight.init("hookRightServo", false, true, Constants.HOOK2_INIT, Constants.HOOK2_POS, Constants.HOOK2_2POS);
    }

    public void initIntake(HardwareMap hardwareMap) {
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake = new IntakeDriver();
        intake.init(intakeMotor);
        intakeServo = new Servo(hardwareMap);
        intakeServo.init("intakeServo", true, true);
    }

    public void initPlane(HardwareMap hardwareMap) {
        planeServo = new Servo(hardwareMap);
        planeServo.init("planeServo", false, false);
    }

    public void initIMU(HardwareMap hardwareMap) {
        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.init(imu);
    }

    public void initArm(HardwareMap hardwareMap) {
        armServoLeft = new Servo(hardwareMap);
        armServoLeft.init("armLeftServo", false, false, Constants.ARM1_POSITION, Constants.ARM1_HIGH, Constants.ARM1_INIT_POS);
        armServoRight = new Servo(hardwareMap);
        armServoRight.init("armRightServo", false, true, Constants.ARM2_HIGH, Constants.ARM2_POSITION, Constants.ARM2_INIT_POS);
    }

    public void initSingleArm(HardwareMap hardwareMap) {
        armServo = new Servo(hardwareMap);
        armServo.init("armSingleServo", false, false, 0, 90, 0);
    }

    public void initClaw(HardwareMap hardwareMap) {
        clawServo = new Servo(hardwareMap);
        clawServo.init("clawServo", false, false, 0, 90, 0);
    }

    public void initAutonomy(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initLift(hardwareMap);
        initIntake(hardwareMap);
        initIMU(hardwareMap);
        initArm(hardwareMap);
    }

    public void initCenterstage(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initLift(hardwareMap);
        initHooks(hardwareMap);
        initIntake(hardwareMap);
        initPlane(hardwareMap);
        initIMU(hardwareMap);
        initArm(hardwareMap);
    }

    public void initIceMode(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initSingleArm(hardwareMap);
        initClaw(hardwareMap);
        initIMU(hardwareMap);
        initArm(hardwareMap);
    }
}
