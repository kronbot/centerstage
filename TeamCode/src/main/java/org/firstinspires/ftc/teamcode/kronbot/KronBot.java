package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.kronbot.utils.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class KronBot {
    public MotorDriver motors;
    public ControlHubGyroscope gyroscope;

    public void Init(HardwareMap hardwareMap) {
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        motors = new MotorDriver(hardwareMap);
        motors.Init(leftRear, leftFront, rightRear, rightFront);

        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.Init(imu);
    }
}
