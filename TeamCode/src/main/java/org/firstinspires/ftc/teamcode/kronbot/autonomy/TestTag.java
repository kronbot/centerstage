package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.TagDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Test Tag", group = Constants.TEST_GROUP)
public class TestTag extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private final TagDetection tagDetection = new TagDetection();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
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