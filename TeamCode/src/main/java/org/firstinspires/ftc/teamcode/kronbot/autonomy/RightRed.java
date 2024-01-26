package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Right Red", group = Constants.MAIN_GROUP)
public class RightRed extends LinearOpMode {
    GameElementDetection detection;
    GameElementDetection.Position position;
    KronBot robot = new KronBot();

    @Override
    public void runOpMode()  throws InterruptedException {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        detection = new GameElementDetection();
        detection.init(hardwareMap, false);

        FtcDashboard.getInstance().startCameraStream(detection.getCamera(), 30);

        while (!opModeIsActive() && !isStopRequested()) {
            position = detection.getPosition();
            telemetry.addData("Position", detection.getPosition());
            telemetry.update();
        }

        detection.close();

        List<TrajectorySequence> trajectory = TrajectoryFactory.createTrajectory(drive, position, robot, telemetry, () -> {sleep(1000); return; }, false, true);

        waitForStart();

        drive.followTrajectorySequence(trajectory.get(0));
        TrajectoryFactory.raiseSlidesClose(robot);
        drive.followTrajectorySequence(trajectory.get(1));
        TrajectoryFactory.resetSlidesClose(robot, () -> { sleep(1000); });
        drive.followTrajectorySequence(trajectory.get(2));

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}