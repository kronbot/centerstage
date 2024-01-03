package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

@Autonomous(name = "Test Game Element", group = Constants.TEST_GROUP)
public class TestGameElement extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private final GameElementDetection gameElementDetection = new GameElementDetection();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gameElementDetection.init(hardwareMap);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            gameElementDetection.detect();
            gameElementDetection.showInfo(telemetry);
            telemetry.update();
        }

        gameElementDetection.close();
    }
}