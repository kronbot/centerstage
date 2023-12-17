package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.LiftDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.ServoDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class KronBot {
    public MotorDriver motors;
    public ServoDriver servos;
    public LiftDriver liftDriver;
    public ControlHubGyroscope gyroscope;

    public Servo armServo1;
    public Servo armServo2;
    public Servo clawServo;
    public Servo intakeServo;
    public Servo planeServo;

    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotorEx leftFront;
    DcMotorEx rightFront;

    public void init(HardwareMap hardwareMap) {
        intakeServo = new Servo(hardwareMap);
        intakeServo.init("intake", false, true);

        clawServo = new Servo(hardwareMap);
        clawServo.init("claw", false, true);

        armServo1 = new Servo(hardwareMap);
        armServo1.init("arm", false, true);
        armServo2 = new Servo(hardwareMap);
        armServo2.init("arm2", false, true);
        planeServo =new Servo(hardwareMap);
        planeServo.init("planeservo",false,false);

        servos = new ServoDriver();
        armServo1.setPosition(Constants.ARM1_INIT_POS);
        armServo2.setPosition(Constants.ARM2_INIT_POS);
        servos.init(armServo1, armServo2, intakeServo, clawServo,planeServo);
        servos.intakeClose();

        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");

        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);

        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.init(imu);

        clawServo.setPosition(Constants.CLAW_INIT_POS);

    }
    public void drive(double frontLeft, double frontRight, double backLeft, double backRight, double power)
    {
        leftFront.setPower(-frontLeft * power);
        rightFront.setPower(frontRight * power);
        leftRear.setPower(-backLeft * power);
        rightRear.setPower(backRight * power);
    }
}
