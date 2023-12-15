package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDES_SPEED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;



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

    public void run() {
        resetButton.updateButton(gamepad.b);
        liftUpButton.updateButton(gamepad.dpad_up);
        liftDownButton.updateButton(gamepad.dpad_down);

        if (resetButton.longPress()) {
            liftMotor.setTargetPosition(0);
            liftMotor2.setTargetPosition(0);
        }else if (liftUpButton.toggle()) {
            liftMotor.setTargetPosition(liftMotor.motor.getCurrentPosition());
            liftMotor2.setTargetPosition(liftMotor2.motor.getCurrentPosition());
            if (currentState > 1)
                currentState--;
        } else if (liftDownButton.toggle()) {
            liftMotor.setTargetPosition(liftMotor.motor.getCurrentPosition());
            liftMotor2.setTargetPosition(liftMotor2.motor.getCurrentPosition());
            if (currentState < 1)
                currentState++;
        }

        switch (currentState) {
            case 0:
                state = LiftPositions.START;
                break;
            case 1:
                state = LiftPositions.INITIAL;
                break;
            case 2:
                state = LiftPositions.TOP;
                break;
        }

        setPower(state.speed);
        setTargetPosition(state.position);
        updatePosition();
    }

    private void updatePosition() {
        liftMotor.updatePosition();
        liftMotor2.updatePosition();
    }

    public void setTargetPosition(double position) {
        liftMotor.setTargetPosition(position);
        liftMotor2.setTargetPosition(position);
    }

    public void setPower(double power) {
        liftMotor.setPower(power);
        liftMotor2.setPower(-power);
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Lift State: ", state);
        telemetry.addData("Motor Mode: ", liftMotor.motor.getMode());

        resetButton.updateButton(gamepad.b);
        liftUpButton.updateButton(gamepad.dpad_up);
        liftDownButton.updateButton(gamepad.dpad_down);

        telemetry.addData("Lift TargetPos: ", liftMotor.targetPosition);
        telemetry.addData("Lift Current Position: ", liftMotor.currentPosition);

        telemetry.addData("Motor Power: ", liftMotor.motor.getPower());
        telemetry.addData("Motor Direction: ", liftMotor.motor.getDirection());
        telemetry.addData("Lift Encoder: ", liftMotor.motor.getCurrentPosition());
        telemetry.addData("Motor Vel: ", liftMotor.motor.getVelocity());
        telemetry.addData("PID Pow: ", liftMotor.getPower());

        telemetry.addData("Position Tolerance: ", tolerance);
        telemetry.addData("Motor Direction: ", liftMotor.direction);
    }
}
