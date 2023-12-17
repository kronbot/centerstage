package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDES_SPEED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class

LiftDriver {
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

    public void init(boolean pid, boolean encoder) {
        liftMotor = new Motor(hardwareMap);
        liftMotor.init("liftMotor", false, pid, true, true, true, encoder);
        if (pid) {
            liftMotor.holdMode(true);
            liftMotor.setTolerance(tolerance);
            liftMotor.setPositionPID(kP, kI, kD);
        }

        liftMotor2 = new Motor(hardwareMap);
        liftMotor2.init("liftMotor2", true, pid, true, true, true, encoder);
        if (pid) {
            liftMotor2.holdMode(true);
            liftMotor2.setTolerance(tolerance);
            liftMotor2.setPositionPID(kP, kI, kD);
        }
    }

    public enum LiftPositions {
        START(0, 0.85),
        INITIAL(500, 0.85);

        public double position = 0;
        public double speed = 0;

        LiftPositions(double value, double speed) {
            this.position = value;
            this.speed = speed;
        }
    }

    boolean manualMode = false;

    Button resetButton = new Button();
    Button liftUpButton = new Button();
    Button liftDownButton = new Button();

    int currentState = 0;

    LiftPositions state = LiftPositions.START;

    public void run() {
        resetButton.updateButton(gamepad.b);
        liftUpButton.updateButton(gamepad.dpad_up);
        liftDownButton.updateButton(gamepad.dpad_down);

        if (resetButton.shortPress()) {
            manualMode = false;
            currentState = 0;
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (liftUpButton.shortPress()) {
            manualMode = false;
            if (currentState > 1)
                currentState--;
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (liftDownButton.shortPress()) {
            manualMode = false;
            if (currentState < 1)
                currentState++;
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        switch (currentState) {
            case 0:
                state = LiftPositions.START;
                break;
            case 1:
                state = LiftPositions.INITIAL;
                break;
        }

        if (liftMotor.targetPosition >= liftMotor.currentPosition - tolerance) {
            if (!manualMode) {
                    manualMode = true;
                    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
        } else {
            if (manualMode) {
                manualMode = false;
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        if (manualMode) {
            setPower(gamepad.right_trigger - gamepad.left_trigger);
        } else {
            setPower(state.speed);
            setTargetPosition(state.position);
            updatePosition();
        }
    }

    public void testSlide() {
        if (gamepad.dpad_up)
            setPower(0.5);
        else if (gamepad.dpad_down)
            setPower(-0.5);
        else
            setPower(0);
    }

    public void run(double power) {
        if (power == 0) setPower(Constants.REST_POWER);
        else setPower(power);
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
        liftMotor2.setPower(power);
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
