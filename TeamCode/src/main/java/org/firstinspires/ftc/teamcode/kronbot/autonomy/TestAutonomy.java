package org.firstinspires.ftc.teamcode.kronbot.autonomy;

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
    private final KronBot robot = new KronBot();
    private MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

    private class MoveAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            drive.actionBuilder(drive.pose)
                    .splineToConstantHeading(new Vector2d(0,10),drive.pose.heading)
                    .build();
            return true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));


        Actions.runBlocking(new MoveAction());
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
