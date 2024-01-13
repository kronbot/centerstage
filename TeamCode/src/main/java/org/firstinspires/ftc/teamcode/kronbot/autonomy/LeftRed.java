package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.RedPixelLeft;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.RedPixelMiddle;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.RedPixelRight;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.io.FileFilter;

@Autonomous(name = "Left Red", group = Constants.MAIN_GROUP)
public class LeftRed extends LinearOpMode {
    GameElementDetection detection;
    GameElementDetection.Position position;
    KronBot robot = new KronBot();

    @Override
    public void runOpMode()  throws InterruptedException {
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        detection = new GameElementDetection();
        detection.init(hardwareMap);

        FtcDashboard.getInstance().startCameraStream(detection.getCamera(), 30);

        while (!opModeIsActive() && !isStopRequested()) {
            position = detection.getPosition();
            telemetry.addData("Position", detection.getPosition());
            telemetry.update();
        }

        detection.close();

        Pose2d pixelPose = new Pose2d(0, 0, 0);
        if (position == GameElementDetection.Position.RIGHT)
            pixelPose = coordinatesConvert(RedPixelRight);
        else if (position == GameElementDetection.Position.MIDDLE)
            pixelPose = coordinatesConvert(RedPixelMiddle);
        else
            pixelPose = coordinatesConvert(RedPixelLeft);

        SequentialAction middlepixel = new SequentialAction(
                drive.actionBuilder(drive.pose).lineToX(pixelPose.position.x).build(),
                (dropPixel) -> {
                    robot.servos.pixel(false);
                    return false;
                },
                drive.actionBuilder(drive.pose).strafeTo(new Vector2d(0, pixelPose.position.y)).build());
        SequentialAction leftRightPixel = new SequentialAction(
                drive.actionBuilder(drive.pose).lineToX(pixelPose.position.x).strafeTo(new Vector2d(pixelPose.position.x,pixelPose.position.y)).build(),
                (dropPixel2) -> {
                    robot.servos.pixel(false);
                    return false;
                },
                drive.actionBuilder(drive.pose).lineToX(pixelPose.position.x+1).build());

        Action middleOfTile = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(new Vector2d(AutonomousConstants.middleOfTile.x, AutonomousConstants.middleOfTile.y),0),Math.PI/2)
                .build();

        waitForStart();
        if(position == GameElementDetection.Position.MIDDLE)
        Actions.runBlocking(middlepixel);
        else {
            Actions.runBlocking(leftRightPixel);
        }
        //Actions.runBlocking(middleOfTile);

        if (isStopRequested()) return;
    }
}