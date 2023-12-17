package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import static java.lang.Double.max;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "Detection Autonomy", group = Constants.MAIN_GROUP)
public class DetectionAutonomy extends LinearOpMode {
    OpenCvCamera camera;
    private final KronBot robot = new KronBot();
    ElapsedTime timer=new ElapsedTime();
    private final GameElementDetection gameElementDetection = new GameElementDetection();
    double acc=0.55;
    double speed=0.60;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gameElementDetection.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });
        int state = 0;

        while (!isStopRequested() && !opModeIsActive()) {
            gameElementDetection.detect();
            gameElementDetection.telemetry(telemetry);

            List<Recognition> gameElementRecognition = gameElementDetection.getGameElement();

            if (gameElementRecognition.size() > 0) {
                Recognition recognition = gameElementRecognition.get(0);
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                if (x < 230)
                    state = 1;
                else
                    state = 2;
            }
            else state = 3;

            telemetry.addData("State", state);
            telemetry.update();
        }

        while (opModeIsActive())
        {
            if(state==1)
            {
                DriveAcc(1, 1,1,1,speed,1.95,acc,0.02);

                while (timer.seconds() < 3) ;
                if (!opModeIsActive())
                    return;

                DriveAcc(-1, 1, 1, -1, speed, 1.4, acc, 0.02);
            }
            else if(state==0)
            {
                DriveAcc(1, 1,1,1,speed,1.8,acc,0.02);
                timer.reset();
                while (timer.seconds() < 3 && opModeIsActive()) ;
                if (!opModeIsActive())
                    return;

                DriveAcc(1, 1, 1, 1, speed, 1.4, acc, 0.02);
            }
            else if(state==2)
            {
                DriveAcc(1,1,1,1,speed,1.0,acc,0.02);
            }
        }
        gameElementDetection.close();
    }
    void DriveAcc(double fl, double fr, double bl, double br, double power, double runTime, double acc, double add) {
        while (timer.seconds() < runTime - runTime / 4) {
            acc = max(acc - add, 0);
            robot.drive(fl, fr, bl, br, power - acc);
            if (!opModeIsActive())
                return;
        }
        while (timer.seconds() < runTime) {
            acc = Math.min(power - 0.2, acc + add);
            robot.drive(fl, fr, bl, br, power - acc);
            if (!opModeIsActive())
                return;
        }
        robot.drive(fl, fr, bl, br, 0);
    }
}