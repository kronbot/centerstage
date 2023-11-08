package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Test Autonomy", group = Constants.MAIN_GROUP)
public class TestAutonomy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        double kP = 0.005, kI = 0, kD = 0.0001;

        TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint angCon = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL * 1.5, DriveConstants.TRACK_WIDTH);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
    }
}
