package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.TrackDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.ServoDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

/**
 * The TeleOP program for the driving on ice with tracks.
 *
 * @version 1.0
 */

@TeleOp(name = "Ice Driving", group = Constants.MAIN_GROUP)
@Disabled
public class IceDrivingOp extends LinearOpMode {
    private final KronBot robot = new KronBot();
    TrackDrive trackDrive;
    ServoDriver servoDriver;
    Gamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        gamepad = gamepad1;

        trackDrive = new TrackDrive(robot, gamepad);

        waitForStart();

        while(!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if(isStopRequested()) return;

        Button reverseButton = new Button();
        Button intakeButton = new Button();

        while(!isStopRequested() && opModeIsActive()) {
            reverseButton.updateButton(gamepad.circle);
            reverseButton.longPress();

            intakeButton.updateButton(gamepad.dpad_up);
            intakeButton.shortPress();

            trackDrive.setReverse(reverseButton.getLongToggle());
            trackDrive.run();

            telemetry.update();
        }
    }
}
