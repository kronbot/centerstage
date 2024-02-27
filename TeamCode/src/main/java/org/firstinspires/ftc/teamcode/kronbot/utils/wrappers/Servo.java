package org.firstinspires.ftc.teamcode.kronbot.utils.wrappers;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * A wrapper for a servo or continuous servo
 *
 * @version 1.0
 */
public class Servo {
    public ServoImplEx servo;
    public CRServoImplEx continuousServo;
    HardwareMap hardwareMap;

    double minBoundary, maxBoundary;
    double increment;

    boolean continuousMode = false;
    boolean isReversed = false;

    public Servo(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init(String name, boolean continuousMode, boolean isReversed) {
        setPWMRange(500, 2500);
        setIncrement(0.001);
        setBoundaries(0, 1);
        setPosition(0);

        this.continuousMode = continuousMode;
        if (continuousMode) continuousServo = hardwareMap.get(CRServoImplEx.class, name);
        else servo = hardwareMap.get(ServoImplEx.class, name);

        setReversed(isReversed);
    }

    public void init(String name, boolean continuousMode, boolean isReversed, double min, double max, double start) {
        setPWMRange(500, 2500);
        setIncrement(0.001);
        setBoundaries(min, max);
        setPosition(start);

        this.continuousMode = continuousMode;
        if (continuousMode) continuousServo = hardwareMap.get(CRServoImplEx.class, name);
        else servo = hardwareMap.get(ServoImplEx.class, name);

        setReversed(isReversed);
    }

    public void setIncrement(double increment) {
        this.increment = increment;
    }

    public void setBoundaries(double min, double max) {
        minBoundary = min;
        maxBoundary = max;
    }

    public void run(boolean action) {
        if (action) setPosition(maxBoundary);
        else setPosition(minBoundary);
    }

    public void runContinuous(boolean action, boolean action2) {
        if (action) setPosition(1);
        else if (action2) setPosition(0);
        else setPosition(0.5);
    }

    public void runIncrement(boolean action, boolean action2) {
        if (action && getPosition() < maxBoundary) setPosition(getPosition() + increment);
        else if (action2 && getPosition() > minBoundary) setPosition(getPosition() - increment);
    }

    public void setPWMRange(double min, double max) {
        if (continuousMode)
            continuousServo.setPwmRange(new PwmControl.PwmRange(min, max));
        else
            servo.setPwmRange(new PwmControl.PwmRange(min, max));
    }

    public void setPosition(double position) {
        if (continuousMode) {
            position *= 2;
            position--;

            continuousServo.setPower(position);
        }
        else {
            if (position > 1)
                position = 1;
            else if (position < 0)
                position = 0;

            servo.setPosition(position);
        }
    }

    public double getPosition() {
        if (!continuousMode)
            return servo.getPosition();
        else return 0;
    }

    public void cutPower() {
        servo.getController().close();
    }

    public void setReversed(boolean isReversed) {
        this.isReversed = isReversed;
    }

    public void setMaxPosition() {
        setPosition(maxBoundary);
    }

    public void setMinPosition() {
        setPosition(minBoundary);
    }
}
