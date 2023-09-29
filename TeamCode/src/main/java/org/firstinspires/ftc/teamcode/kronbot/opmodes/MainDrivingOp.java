package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.kronbot.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Gyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@Config
@TeleOp(name = "Main Driving", group = Constants.mainGroup)
public class MainDrivingOp extends LinearOpMode {
    MotorDriver motors;
    ControlHubGyroscope gyroscope;

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;

    Servo clawServo;
    Servo armServo;
    Servo planeServo;

    public static double BreakPower = 0;
    public static double Ticks = -400;
    public static double Offset = 0.2;
    DcMotor arm;

    @Override
    public void runOpMode() throws InterruptedException {
        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        motors = new MotorDriver(hardwareMap);
        motors.Init();

        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.Init();

        robotCentricDrive = new RobotCentricDrive(motors, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(motors, drivingGamepad, gyroscope);

        clawServo = new Servo(hardwareMap);
        clawServo.Init("claw", false, false);
        clawServo.setPosition(1);

        armServo = new Servo(hardwareMap);
        armServo.Init("arm", false, false);
        armServo.setPosition(1);

        planeServo = new Servo(hardwareMap);
        planeServo.Init("plane", false, false);
        planeServo.setPosition(1);

        arm = hardwareMap.get(DcMotorImpl.class, "armMotor");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        Button driveModeButton = new Button();
        Button reverseButton = new Button();
        Button clawButton = new Button();
        Button planeButton = new Button();

        while (opModeIsActive() && !isStopRequested()) {
            driveModeButton.updateButton(drivingGamepad.x);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.b);
            reverseButton.shortPress();

            clawButton.updateButton(drivingGamepad.a);
            clawButton.shortPress();

            planeButton.updateButton(drivingGamepad.y);
            planeButton.shortPress();

            robotCentricDrive.setReverse(reverseButton.getShortToggle());

            if (!driveModeButton.getLongToggle()) {
                robotCentricDrive.run();
                robotCentricDrive.showInfo(telemetry);
            } else {
                fieldCentricDrive.run();
                fieldCentricDrive.showInfo(telemetry);
            }

            if (clawButton.getShortToggle()) {
                telemetry.addData("Position", clawServo.getPosition());
                clawServo.setPosition(1 - clawServo.getPosition());
                clawButton.resetToggles();
            }

            if (planeButton.getShortToggle()) {
                telemetry.addData("Position", planeServo.getPosition());
                planeServo.setPosition(1 - planeServo.getPosition());
                planeButton.resetToggles();
            }

            if (drivingGamepad.dpad_up) {
                armServo.setPosition(armServo.getPosition() - 0.05);
            } else if (drivingGamepad.dpad_down) {
                armServo.setPosition(armServo.getPosition() + 0.05);
            }

            if (drivingGamepad.right_trigger > 0.1) {
                arm.setPower(drivingGamepad.right_trigger * Offset  );
            } else if (drivingGamepad.left_trigger > 0.1) {
                arm.setPower(-drivingGamepad.left_trigger * Offset);
            } else {
                if (arm.getCurrentPosition() > Ticks)
                    arm.setPower(BreakPower);
                else
                    arm.setPower(-BreakPower);
            }

            telemetry.update();
        }
    }
}
