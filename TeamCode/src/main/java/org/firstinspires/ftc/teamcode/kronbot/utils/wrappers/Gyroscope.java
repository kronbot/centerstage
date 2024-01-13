package org.firstinspires.ftc.teamcode.kronbot.utils.wrappers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.kronbot.utils.pid.ControllerPID;

/**
 * A wrapper for the gyroscope
 *
 * @version 1.0
 */
public class Gyroscope {
    BNO055IMU imu;
    HardwareMap hardwareMap;

    public double firstHeading = 0;
    public double firstLateral = 0;
    public double firstForward = 0;

    boolean firstAngles = false;

    public Gyroscope(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init(BNO055IMU imu) {
        this.imu = imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu.initialize(parameters);
    }

    public BNO055IMU getIMU() {
        return imu;
    }

    Orientation angularOrientation;
    AngularVelocity angularVelocity;

    double updateInterval = 1;
    ElapsedTime orientationUpdateTimer = new ElapsedTime();

    public void updateOrientation() {
        if (orientationUpdateTimer.milliseconds() > updateInterval) {
            angularOrientation = imu.getAngularOrientation();
            orientationUpdateTimer.reset();
        }
    }

    ElapsedTime velocityUpdateTimer = new ElapsedTime();

    public void updateVelocity() {
        if (velocityUpdateTimer.milliseconds() > updateInterval) {
            angularVelocity = imu.getAngularVelocity();
            velocityUpdateTimer.reset();
        }
    }

    public void setUpdateInterval(double interval) {
        updateInterval = interval;
    }

    public double getHeading() {
        return angularOrientation.firstAngle;
    }

    public double getLateralAngle() {
        return angularOrientation.secondAngle;
    }

    public double getForwardAngle() {
        return angularOrientation.thirdAngle;
    }

    public void initFirstAngles() {
        ElapsedTime initTime = new ElapsedTime();
        while (firstHeading == firstLateral && firstLateral == firstForward && initTime.milliseconds() < 2000) {
            updateOrientation();
            firstAngles = true;
            firstHeading = getHeading();
            firstLateral = getLateralAngle();
            firstForward = getForwardAngle();
        }
    }

    ControllerPID secondAnglePID = new ControllerPID(0.01, 0, 0);
    ControllerPID thirdAnglePID = new ControllerPID(0.01, 0, 0);

    public void showInfo(Telemetry telemetry) {
        double secondAngle = secondAnglePID.calculate(0, angularOrientation.secondAngle);
        double thirdAngle = thirdAnglePID.calculate(0, angularOrientation.thirdAngle);

        telemetry.addLine("---ANGLES---");
        telemetry.addData("First Angle", angularOrientation.firstAngle);
        telemetry.addData("Second Angle", angularOrientation.secondAngle);
        telemetry.addData("Third Angle", angularOrientation.thirdAngle);

        telemetry.addLine("---PID---");
        telemetry.addData("Second Angle PID", secondAngle);
        telemetry.addData("Third Angle PID", thirdAngle);

        telemetry.addLine("---VELOCITY---");
        telemetry.addData("X Rotation Rate", angularVelocity.xRotationRate);
        telemetry.addData("Y Rotation Rate", angularVelocity.yRotationRate);
        telemetry.addData("Z Rotation Rate", angularVelocity.zRotationRate);
    }
}
