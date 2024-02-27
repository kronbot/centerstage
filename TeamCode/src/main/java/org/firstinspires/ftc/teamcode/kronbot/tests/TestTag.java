package org.firstinspires.ftc.teamcode.kronbot.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.detection.TagDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

@Disabled
@Autonomous(name = "Test Tag", group = Constants.TEST_GROUP)
public class TestTag extends LinearOpMode {
    private final TagDetection tagDetection = new TagDetection();

    @Override
    public void runOpMode() throws InterruptedException {
        tagDetection.init(hardwareMap);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            tagDetection.detect();
            tagDetection.telemetry(telemetry);
            telemetry.update();
        }

        tagDetection.close();
    }
}