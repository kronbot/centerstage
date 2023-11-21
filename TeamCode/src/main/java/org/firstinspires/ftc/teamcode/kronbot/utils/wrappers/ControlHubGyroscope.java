package org.firstinspires.ftc.teamcode.kronbot.utils.wrappers;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.kronbot.utils.ControllerPID;

/**
 * A wrapper for the gyroscope on the control hub
 *
 * @version 1.0
 */
public class ControlHubGyroscope {
    BHI260IMU imu;
    HardwareMap hardwareMap;

    public double firstHeading = 0;
    public double firstLateral = 0;
    public double firstForward = 0;

    public boolean firstAngles = false;

    public ControlHubGyroscope(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init(BHI260IMU imu) {
        this.imu = hardwareMap.get(BHI260IMU.class, "imu");

        ImuOrientationOnRobot imuOrientationOnRobot =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(imuOrientationOnRobot);

        this.imu.initialize(parameters);
    }

    Orientation angularOrientation;
    AngularVelocity angularVelocity;

    double updateInterval = 1;
    ElapsedTime orientationUpdateTimer = new ElapsedTime();

    public void updateOrientation() {
        if (orientationUpdateTimer.milliseconds() > updateInterval) {
            angularOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            orientationUpdateTimer.reset();
        }
    }

    ElapsedTime velocityUpdateTimer = new ElapsedTime();

    public void updateVelocity() {
        if (velocityUpdateTimer.milliseconds() > updateInterval) {
            angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
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

    public void resetHeading() {
        imu.resetYaw();
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
