package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_INIT_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
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
        Button hook2Button = new Button();

        boolean hooks = false;

        while (opModeIsActive() && !isStopRequested()) {
            driveModeButton.updateButton(drivingGamepad.square);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.circle);
            reverseButton.shortPress();

            hookButton.updateButton(utilityGamepad.circle);
            hookButton.longPress();
            hookButton.shortPress();

            hook2Button.updateButton(utilityGamepad.dpad_left);
            hook2Button.shortPress();

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

            if (hookButton.getLongToggle()) {
                hooks = true;
                hookButton.resetToggles();
                hook2Button.resetToggles();
            } else if (hookButton.getShortToggle()) {
                hooks = false;
                hookButton.resetToggles();
                hook2Button.resetToggles();
            }

            if (hook2Button.getShortToggle() && hooks) {
                robot.servos.hook2(hook2Button.getShortToggle());
            } else {
                robot.servos.hook(hooks);
            }


            robot.servos.plane(planeButton.getLongToggle());

            robot.intake.drive(utilityGamepad.dpad_up, utilityGamepad.dpad_down);
            if (utilityGamepad.dpad_up) robot.servos.intakeSpinUp(utilityGamepad.dpad_up);
            else robot.servos.intakeSpinDown(utilityGamepad.cross);

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
