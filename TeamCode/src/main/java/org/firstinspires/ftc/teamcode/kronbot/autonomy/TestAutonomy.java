package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

@Autonomous(name = "Test Autonomy", group = Constants.TEST_GROUP)
public class TestAutonomy extends LinearOpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(0, 0, 0));
//
//        TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
//        TrajectoryVelocityConstraint angCon = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL * 1.5, DriveConstants.TRACK_WIDTH);
//
//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//            .forward(55.6)
//            .build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

//        drive.followTrajectorySequenceAsync(trajectory);
//
//        while (drive.isBusy())
//            drive.update();
    }
}
