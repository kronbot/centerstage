package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.ServoDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class KronBot {
    public MotorDriver motors;
    public ServoDriver servos;
    public ControlHubGyroscope gyroscope;

    public Servo armServo1;
    public Servo armServo2;
    public Servo clawServo;
    public Servo intakeServo;

    public void init(HardwareMap hardwareMap) {
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");

        intakeServo = new Servo(hardwareMap);
        intakeServo.init("intake", false, true);

        clawServo = new Servo(hardwareMap);
        clawServo.init("claw", false, true);

        armServo1 = new Servo(hardwareMap);
        armServo1.init("arm", false, true);
        armServo2 = new Servo(hardwareMap);
        armServo2.init("arm2", false, true);

        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);

        servos = new ServoDriver();
        clawServo.setPosition(0);
        armServo1.setPosition(0);
        armServo2.setPosition(1);
        servos.init(armServo1, armServo2, intakeServo, clawServo);
        servos.intake(true);

        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.init(imu);
    }
}
