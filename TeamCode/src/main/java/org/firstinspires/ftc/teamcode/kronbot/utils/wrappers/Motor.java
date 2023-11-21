package org.firstinspires.ftc.teamcode.kronbot.utils.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

import org.firstinspires.ftc.teamcode.kronbot.utils.ControllerPID;

/**
 * A wrapper for a DC motor with encoder
 *
 * @version 1.0
 */
public class Motor {
    public DcMotorEx motor;
    public DcMotorEx motor2;
    HardwareMap hardwareMap;
    VoltageSensor voltageSensor;

    public Motor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init(String name, String name2, boolean isReversed, boolean velocityPIDFMode, boolean positionPIDMode, boolean brakes) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor2 = hardwareMap.get(DcMotorEx.class, name2);

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.setDirection(isReversed);
        this.setPositionPIDMode(positionPIDMode);
        this.setVelocityPIDFMode(velocityPIDFMode);
        this.setBrakes(brakes);
    }

    PIDCoefficients positionCoefficients = new PIDCoefficients(0.007, 0.003, 0.001);
    PIDCoefficients velocityCoefficients = new PIDCoefficients(0.0005, 0.0000, 0.00005);
    double maxVel = 1000;

    PIDFController posController = new PIDFController(positionCoefficients);
    PIDFController velController = new PIDFController(velocityCoefficients);

    public double calcPos;
    public double power;

    public void updatePosition() {
        currentPosition = motor.getCurrentPosition();

        posController.setTargetPosition(calcPos);

        power = posController.update(currentPosition);
        motor.setPower((power + 0.15) * 0.9);
        motor2.setPower((power + 0.15) * 0.9);
    }

    public void resetPID() {
        PIDFController posController = new PIDFController(positionCoefficients);
    }

    boolean voltageCompensated = false;
    double voltageCompensation = 1;

    public void setVoltageCompensated(boolean voltageCompensated) {
        this.voltageCompensated = voltageCompensated;
    }

    public double getVoltageCompensation() {
        return (12 / voltageSensor.getVoltage());
    }

    boolean positionPIDMode = false;
    ControllerPID positionPID = new ControllerPID(0, 0, 0);

    public void setPositionPIDMode(boolean positionPIDMode) {
        this.positionPIDMode = positionPIDMode;
    }

    public void setPositionPID(double kP, double kI, double kD) {
        positionPID = new ControllerPID(kP, kI, kD);
    }

    public double targetPosition = 0;
    public double currentPosition = 0;

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    double tolerance = 0;

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double direction = 1;

    double getDirection() {
        if (currentPosition <= targetPosition)
            return 1;
        else
            return -1;
    }

    public boolean isBusy() {
        return (Math.abs(targetPosition - currentPosition) > tolerance);
    }

    public void setPower(double power) {
        motor.setPower(power);
        motor2.setPower(power);
    }

    boolean hold = false;
    double gravityCounter = 0.1;

    public void holdMode(boolean holdMode) {
        hold = holdMode;
    }

    double speedMultiplier = 1;

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public void setVelocityPIDFMode(boolean velocityPIDMode) {
        if (velocityPIDMode)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setVelocityPIDF(double kP, double kI, double kD, double kF) {
        motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public void setDirection(boolean isReversed) {
        if (isReversed)
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double encoderDirection = 1;

    @Deprecated
    public void setEncoderDirection(double encoderDirection) {
        this.encoderDirection = encoderDirection;
    }

    public void setBrakes(boolean brakes) {
        if (brakes)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
