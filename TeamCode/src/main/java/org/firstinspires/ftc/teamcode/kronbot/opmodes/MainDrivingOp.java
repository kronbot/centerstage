package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_INIT_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.LiftDriver;
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

        while (opModeIsActive() && !isStopRequested()) {
            driveModeButton.updateButton(drivingGamepad.square);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.circle);
            reverseButton.shortPress();

            hookButton.updateButton(utilityGamepad.cross);
            hookButton.longPress();

            robotCentricDrive.setReverse(reverseButton.getShortToggle());
            robot.servos.hook(hookButton.getLongToggle());

            robot.intake.drive(utilityGamepad.dpad_up, utilityGamepad.dpad_down);
            robot.servos.intake(utilityGamepad.dpad_up);

            robot.hook.drive(utilityGamepad.left_bumper, utilityGamepad.right_bumper);
            robot.lift.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
            if (robot.lift.getCurrentPosition() > LIFT_INIT_POSITION)
                robot.servos.arm(true);
            else
                robot.servos.arm(false);

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
