package org.firstinspires.ftc.teamcode.kronbot.tests;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Test Autonomy", group = Constants.TEST_GROUP)
public class TestAutonomy extends LinearOpMode {
    KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
            .strafeRight(10)
            .forward(5)
            .build();

        waitForStart();

        drive.followTrajectory(trajectory);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
