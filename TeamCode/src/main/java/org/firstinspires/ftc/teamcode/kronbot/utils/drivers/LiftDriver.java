package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.REST_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDES_SPEED;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;



public class LiftDriver {
    double kP = 0.005, kI = 0, kD = 0.0001;
    Gamepad gamepad;
    HardwareMap hardwareMap;
    Motor liftMotor;
    Motor liftMotor2;

    double tolerance = 15;

    public LiftDriver(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        liftMotor = new Motor(hardwareMap);
        liftMotor.init("liftMotor", false, false, true, false, true);
        liftMotor.holdMode(true);
        liftMotor.setTolerance(tolerance);
        liftMotor.setPositionPID(kP, kI, kD);

        liftMotor2 = new Motor(hardwareMap);
        liftMotor2.init("liftMotor2", true, false, true, false, true);
        liftMotor2.holdMode(true);
        liftMotor2.setTolerance(tolerance);
        liftMotor2.setPositionPID(kP, kI, kD);

        setPower(SLIDES_SPEED);
    }

    public enum LiftPositions {
        START(0, 0.85, 0),
        INITIAL(125, 0.85, 0),
        TOP(2000, 0.85, 0);

        public double position = 0;
        public double speed = 0;
        public double axlePos = 0;

        LiftPositions(double value, double speed, double axlePos) {
            this.position = value;
            this.speed = speed;
            this.axlePos = axlePos;
        }
    }
    int toggleStates = 0;
    Button resetButton = new Button();
    Button liftUpButton = new Button();
    Button liftDownButton = new Button();

    int currentState = 0;

    LiftPositions state = LiftPositions.START;

    public void setTargetPosition(int position) {
        liftMotor.setTargetPosition(position);
        liftMotor2.setTargetPosition(position);
    }

    public void setPower(double power) {
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    public void run()
    {
        resetButton.updateButton(gamepad.b);
        liftUpButton.updateButton(gamepad.dpad_up);
        liftDownButton.updateButton(gamepad.dpad_down);

        if (liftUpButton.shortPress()) {
            setPower(SLIDES_SPEED);
            liftMotor.setTargetPosition(liftMotor.motor.getCurrentPosition());
            liftMotor2.setTargetPosition(liftMotor2.motor.getCurrentPosition());
        } else if (liftDownButton.shortPress()) {
            setPower(-SLIDES_SPEED);
            liftMotor.setTargetPosition(liftMotor.motor.getCurrentPosition());
            liftMotor2.setTargetPosition(liftMotor2.motor.getCurrentPosition());
        }

        switch (toggleStates) {
            case 1:
                state = LiftPositions.INITIAL;
                break;
            case 2:
                state = LiftPositions.TOP;
                break;
            default:
                state = LiftPositions.START;
                break;
        }

        if (liftDownButton.press())
                liftMotor.setPower(-0.7);
        else if (liftUpButton.press())
                liftMotor.setPower(0.9);
            else
                liftMotor.setPower(REST_POWER);
            liftMotor.setTargetPosition(liftMotor.motor.getCurrentPosition());
        }

//        if (resetButton.longPress()) {
//
//

//    public void resetLift() {
//        setTargetPosition.
//    }
}
