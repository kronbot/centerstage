package org.firstinspires.ftc.teamcode.kronbot.tests;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Test Autonomy", group = Constants.TEST_GROUP)
public class TestAutonomy extends LinearOpMode {

    KronBot robot = new KronBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .lineToX(30)
//                        .build());

        Action hatz = drive.actionBuilder(drive.pose)
                .lineToX(30)
                .turn(Math.toRadians(180))
                .build();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(40)
                        .turnTo(Math.toRadians(180))
                        .build()
        );

//        Actions.runBlocking(hatz);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
