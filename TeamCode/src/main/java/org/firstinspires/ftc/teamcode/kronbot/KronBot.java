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

    public void init(HardwareMap hardwareMap) {
        intakeServo = new Servo(hardwareMap);
        intakeServo.init("intake", false, true);

        clawServo = new Servo(hardwareMap);
        clawServo.init("claw", false, true);

        armServo1 = new Servo(hardwareMap);
        armServo1.init("arm", false, true);
        armServo2 = new Servo(hardwareMap);
        armServo2.init("arm2", false, true);

        servos = new ServoDriver();
        armServo1.setPosition(Constants.ARM1_INIT_POS);
        armServo2.setPosition(Constants.ARM2_INIT_POS);
        servos.init(armServo1, armServo2, intakeServo, clawServo);
        servos.intakeClose();

        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");

        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);

        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.init(imu);

        clawServo.setPosition(Constants.CLAW_INIT_POS);
    }
}
