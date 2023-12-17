package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import static java.lang.Double.max;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

import java.util.List;

@Autonomous(name = "Test Game Element", group = Constants.TEST_GROUP)
public class TestGameElement extends LinearOpMode {
    private final KronBot robot = new KronBot();
    ElapsedTime timer = new ElapsedTime();
    private final GameElementDetection gameElementDetection = new GameElementDetection();
    double acc = 0.55;
    double speed = 0.60;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gameElementDetection.init(hardwareMap);

        while (!isStopRequested() && !opModeIsActive()) {
            gameElementDetection.detect();
            gameElementDetection.telemetry(telemetry);
            telemetry.update();
        }
            gameElementDetection.close();
        }
//    void DriveAcc(double fl, double fr, double bl, double br, double power, double runTime, double acc, double add) {
//        while (timer.seconds() < runTime - runTime / 4) {
//            acc = max(acc - add, 0);
//            robot.drive(fl, fr, bl, br, power - acc);
//            if (!opModeIsActive())
//                return;
//        }
//        while (timer.seconds() < runTime) {
//            acc = Math.min(power - 0.2, acc + add);
//        }
//    }
}