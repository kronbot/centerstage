package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.HangDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.IntakeDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.LiftDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.ServoDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class KronBot {
    public MotorDriver motors;
    public ServoDriver servos;
    public IntakeDriver intake;
    public HangDriver hook;
    public LiftDriver lift;

    public ControlHubGyroscope gyroscope;

    public void init(HardwareMap hardwareMap) {
        Servo armServo1 = new Servo(hardwareMap);
        armServo1.init("arm", false, true);
        Servo armServo2 = new Servo(hardwareMap);
        armServo2.init("arm2", false, true);
        Servo pixelServo = new Servo(hardwareMap);
        pixelServo.init("pixel", false, true);
        Servo hookServo1 = new Servo(hardwareMap);
        hookServo1.init("hook", false, true);
        Servo hookServo2 = new Servo(hardwareMap);
        hookServo2.init("hook2", false, true);
        servos = new ServoDriver();
        armServo1.setPosition(Constants.ARM1_INIT_POS);
        armServo2.setPosition(Constants.ARM2_INIT_POS);
        servos.init(armServo1, armServo2, pixelServo, hookServo1, hookServo2);
        servos.pixel(true);
        servos.hook(false);
        servos.arm(false);

        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intake = new IntakeDriver();
        intake.init(intakeMotor);

        DcMotorEx hangMotor = hardwareMap.get(DcMotorEx.class, "hangLeft");
        DcMotorEx hangMotor2 = hardwareMap.get(DcMotorEx.class, "hangRight");
        hook = new HangDriver();
        hook.init(hangMotor, hangMotor2);

        Motor liftMotor = new Motor(hardwareMap);
        lift = new LiftDriver();
        lift.init(liftMotor, false);

        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.init(imu);
    }
}
