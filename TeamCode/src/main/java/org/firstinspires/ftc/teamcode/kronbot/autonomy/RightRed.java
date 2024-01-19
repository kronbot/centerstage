package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import static org.firstinspires.ftc.teamcode.kronbot.utils.ActionFactory.ActionBuilder;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.Back;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.ClosePark;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelForward;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelLeft;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelMiddle;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelRight;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Right Red", group = Constants.MAIN_GROUP)
public class RightRed extends LinearOpMode {
    GameElementDetection detection;
    GameElementDetection.Position position;
    KronBot robot = new KronBot();

    @Override
    public void runOpMode()  throws InterruptedException {
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

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

        waitForStart();

        SequentialAction action = ActionBuilder(robot, drive, false, true, position, () -> {
            sleep(1000);
        });

        Actions.runBlocking(action);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}