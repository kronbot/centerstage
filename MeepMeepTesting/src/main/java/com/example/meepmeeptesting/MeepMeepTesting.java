package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        boolean isBlue = true;
        boolean isClose = false;

        Pose2d startPose;
        int multiplier = isBlue ? -1 : 1;

        if (isBlue) {
            if (isClose) startPose = new Pose2d(12, 70 - 15/2, Math.toRadians(90));
            else startPose = new Pose2d(-35, 70 - 15/2, Math.toRadians(90));
        } else {
            if (isClose) startPose = new Pose2d(12, -70 + 15/2, Math.toRadians(270));
            else startPose = new Pose2d(-35, -70 + 15/2, Math.toRadians(270));
        }

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17.7)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(startPose.getX(), startPose.getY() + 10 * multiplier))
                    .splineTo(new Vector2d(startPose.getX() - 4 * multiplier, startPose.getY() + 30 * multiplier), Math.toRadians(40 * multiplier))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(startPose.getX(), -59 * multiplier), Math.toRadians(270 * multiplier))
                    .turn(Math.toRadians(-90 * multiplier))
                    .lineToConstantHeading(new Vector2d(15, -59 * multiplier))
                    .splineTo(new Vector2d(45, -35 * multiplier), 0)
                    .build()
            );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}