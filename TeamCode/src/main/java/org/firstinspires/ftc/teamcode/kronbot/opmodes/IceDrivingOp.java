package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.components.TrackDrive;

@TeleOp(name = "iceOpMode")
public class IceDrivingOp extends LinearOpMode {
    KronBot robot = new KronBot();
    Gamepad gamepad;

    TrackDrive trackDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gamepad = gamepad1;

        trackDrive = new TrackDrive(robot, gamepad,hardwareMap);
        trackDrive.init();
        waitForStart();

        if(isStopRequested()) return;

        while(!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        while(!isStopRequested() && opModeIsActive()) {
            trackDrive.run();
        }


    }
}
