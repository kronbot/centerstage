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
        robot.initCenterstage(hardwareMap);

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

        Button driveModeButton = new Button();
        Button reverseButton = new Button();
        Button hookButton = new Button();
        Button armButton = new Button();
        Button planeButton = new Button();
        Button armHighButton = new Button();
        Button hook2Button = new Button();

        boolean areHooksActivated = false;

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Hooks
            hookButton.updateButton(utilityGamepad.circle);
            hookButton.longPress();
            hookButton.shortPress();
            hook2Button.updateButton(utilityGamepad.dpad_left);
            hook2Button.shortPress();
            if (hookButton.getLongToggle()) {
                areHooksActivated = true;
                hookButton.resetToggles();
                hook2Button.resetToggles();
            } else if (hookButton.getShortToggle()) {
                areHooksActivated = false;
                hookButton.resetToggles();
                hook2Button.resetToggles();
            }

            robot.hook.drive(utilityGamepad.left_bumper, utilityGamepad.right_bumper);
            if (hook2Button.getShortToggle() && areHooksActivated) {
                robot.hookServoLeft.setMaxPosition();
                robot.hookServoRight.setMaxPosition();
            } else {
                robot.hookServoLeft.run(areHooksActivated);
                robot.hookServoRight.run(areHooksActivated);
            }

            // Plane
            planeButton.updateButton(utilityGamepad.triangle);
            planeButton.longPress();
            robot.planeServo.run(planeButton.getLongToggle());

            // Intake
            robot.intake.drive(utilityGamepad.dpad_up, utilityGamepad.dpad_down);
            robot.intakeServo.runContinuous(utilityGamepad.dpad_up, utilityGamepad.cross);

            // Lift
            robot.lift.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);

//             Arm
            if (robot.lift.getCurrentPosition() > LIFT_INIT_POSITION) {
                armButton.updateButton(utilityGamepad.square);
                armButton.shortPress();
                armHighButton.updateButton(utilityGamepad.dpad_right);
                armHighButton.longPress();
            }
            if (!armHighButton.getLongToggle()) {
                robot.armServoLeft.run(!armButton.getShortToggle());
                robot.armServoRight.run(!armButton.getShortToggle());
            } else if (armHighButton.getLongToggle()) {
                robot.armServoLeft.setPosition(Constants.ARM1_HIGH);
                robot.armServoRight.setPosition(Constants.ARM2_HIGH);
            }

            // Wheels
            driveModeButton.updateButton(drivingGamepad.square);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.circle);
            reverseButton.shortPress();
            robotCentricDrive.setReverse(reverseButton.getShortToggle());
            if (!driveModeButton.getLongToggle()) {
                robotCentricDrive.run();
                robotCentricDrive.telemetry(telemetry);
            } else {
                fieldCentricDrive.run();
                fieldCentricDrive.telemetry(telemetry);
            }

            telemetry.update();
        }
    }
}
