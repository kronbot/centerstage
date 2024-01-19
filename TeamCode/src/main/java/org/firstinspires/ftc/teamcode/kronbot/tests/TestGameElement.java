package org.firstinspires.ftc.teamcode.kronbot.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

/**
 * A test for the game element detection pipeline
 *
 * @verion 1.0
 */
@Disabled
@Autonomous(name = "Test Game Element", group = Constants.TEST_GROUP)
public class TestGameElement extends LinearOpMode {
    GameElementDetection detection;

    @Override
    public void runOpMode()  throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        detection = new GameElementDetection();
        detection.init(hardwareMap, true);

        FtcDashboard.getInstance().startCameraStream(detection.getCamera(), 30);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Position", detection.getPosition());
            telemetry.update();
        }

        waitForStart();

        detection.close();
    }
}