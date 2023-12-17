package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

import java.util.List;

@Autonomous(name = "Detection Autonomy", group = Constants.MAIN_GROUP)
public class DetectionAutonomy extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private final GameElementDetection gameElementDetection = new GameElementDetection();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gameElementDetection.init(hardwareMap);

        int state;

        while (!isStopRequested() && !opModeIsActive()) {
            gameElementDetection.detect();
            gameElementDetection.telemetry(telemetry);

            List<Recognition> gameElementRecognition = gameElementDetection.getGameElement();

            if (gameElementRecognition.size() > 0) {
                Recognition recognition = gameElementRecognition.get(0);
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                if (x < 320) state = 1;
                else state = 2;
            } else state = 0;

            telemetry.addData("State", state);

            telemetry.update();
        }

        while (opModeIsActive()) {

        }

        gameElementDetection.close();
    }
}