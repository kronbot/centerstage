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

    boolean continuousMode = false;
    boolean isReversed = false;

    public Servo(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init(String name, boolean continuousMode, boolean isReversed) {
        this.continuousMode = continuousMode;
        if (continuousMode)
            continuousServo = hardwareMap.get(CRServoImplEx.class, name);
        else
            servo = hardwareMap.get(ServoImplEx.class, name);
        setReversed(isReversed);
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

    public void setReversed(boolean isReversed) {
        this.isReversed = isReversed;
    }
}
