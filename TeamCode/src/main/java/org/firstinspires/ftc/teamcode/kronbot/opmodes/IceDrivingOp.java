package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.TrackDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

/**
 * The TeleOP program for the driving on ice with tracks.
 *
 * @version 1.0
 */

@TeleOp(name = "Ice Driving", group = Constants.MAIN_GROUP)
public class IceDrivingOp extends LinearOpMode {
    private final KronBot robot = new KronBot();
    TrackDrive trackDrive;
    Gamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initIceMode(hardwareMap);

        gamepad = gamepad1;

        trackDrive = new TrackDrive(robot, gamepad);

        Button reverseButton = new Button();
        Button intakeButton = new Button();

        while(!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if(isStopRequested()) return;

        while(!isStopRequested() && opModeIsActive()) {
//            intakeButton.updateButton(gamepad.cross);
//            intakeButton.shortPress();
//            robot.intakeServo.run(intakeButton.getShortToggle());

            robot.clawServo.runIncrement(gamepad.dpad_up, gamepad.dpad_down);

            robot.armServoLeft.runIncrement(gamepad.right_bumper, gamepad.left_bumper);
            if(gamepad.right_bumper && robot.armServoLeft.getPosition() != Constants.ARM1_HIGH) robot.armServoLeft.setPosition(Constants.ARM1_HIGH);
            if(gamepad.left_bumper && robot.armServoLeft.getPosition() != Constants.ARM1_POSITION) robot.armServoLeft.setPosition(Constants.ARM1_POSITION);

            reverseButton.updateButton(gamepad.circle);
            reverseButton.longPress();
            trackDrive.setReverse(reverseButton.getLongToggle());
            trackDrive.run();

            telemetry.update();
        }
    }
}
