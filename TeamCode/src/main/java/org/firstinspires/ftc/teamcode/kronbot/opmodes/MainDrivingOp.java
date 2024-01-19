package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.MOTOR_SLEEP_TIME;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Main Driving", group = Constants.MAIN_GROUP)
public class MainDrivingOp extends LinearOpMode {
    private final KronBot robot = new KronBot();
    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;
    Gamepad drivingGamepad;
    Gamepad utilityGamepad;

    boolean moved = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        Button driveModeButton = new Button();
        Button reverseButton = new Button();
        Button hookButton = new Button();
        Button armButton = new Button();
        Button planeButton = new Button();
        Button armHighButton = new Button();
        Button resetButton = new Button();

        while (opModeIsActive() && !isStopRequested()) {
            driveModeButton.updateButton(drivingGamepad.square);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.circle);
            reverseButton.shortPress();

            hookButton.updateButton(utilityGamepad.circle);
            hookButton.longPress();

            if (robot.lift.getCurrentPosition() > LIFT_INIT_POSITION) {
                armButton.updateButton(utilityGamepad.square);
                armButton.shortPress();
            }

            planeButton.updateButton(utilityGamepad.triangle);
            planeButton.longPress();

            resetButton.updateButton(utilityGamepad.circle);
            resetButton.shortPress();

            if (robot.lift.getCurrentPosition() > LIFT_INIT_POSITION) {
                armHighButton.updateButton(utilityGamepad.dpad_right);
                armHighButton.longPress();
            }

            robotCentricDrive.setReverse(reverseButton.getShortToggle());

//            if (hookButton.getLongToggle()) {
//                if (!moved) {
//                    robot.hook.drive(true, false);
//                    sleep(MOTOR_SLEEP_TIME);
//                    robot.hook.drive(false, false);
//                    moved = true;
//                }
//                robot.servos.hook(hookButton.getLongToggle());
//            } else moved = false;

            robot.servos.hook(hookButton.getLongToggle());


            robot.servos.plane(planeButton.getLongToggle());

            robot.intake.drive(utilityGamepad.dpad_up, utilityGamepad.dpad_down);
            if (utilityGamepad.dpad_up) robot.servos.intakeSpinUp(utilityGamepad.dpad_up);
            else robot.servos.intakeSpinDown(utilityGamepad.cross);

//            if (resetButton.getShortToggle()) {
//                robot.lift.setTargetPosition(0);
//                robot.servos.arm(false);
//
//                armButton.resetToggles();
//                resetButton.resetToggles();
//            }

            robot.hook.drive(utilityGamepad.left_bumper, utilityGamepad.right_bumper);
            robot.lift.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
            if (!armHighButton.getLongToggle())
                robot.servos.arm(armButton.getShortToggle());
            else
                robot.servos.high(armHighButton.getLongToggle());

            if (!driveModeButton.getLongToggle()) {
                robotCentricDrive.run();
                robotCentricDrive.telemetry(telemetry);
            } else {
                fieldCentricDrive.run();
                fieldCentricDrive.telemetry(telemetry);
            };
            telemetry.update();
        }
    }
}
