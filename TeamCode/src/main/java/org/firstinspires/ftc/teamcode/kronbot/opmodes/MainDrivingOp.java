package org.firstinspires.ftc.teamcode.kronbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.LiftDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Main Driving", group = Constants.MAIN_GROUP)
public class MainDrivingOp extends LinearOpMode {
    private final KronBot robot = new KronBot();
    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;
    LiftDriver liftDriver;
    Gamepad drivingGamepad;
    Gamepad utilityGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);
        liftDriver = new LiftDriver(hardwareMap, utilityGamepad);
        liftDriver.init(false, false);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        Button driveModeButton = new Button();
        Button reverseButton = new Button();
        Button intakeButton = new Button();
        Button planeButton=new Button();

        while (opModeIsActive() && !isStopRequested()) {
            driveModeButton.updateButton(drivingGamepad.x);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.b);
            reverseButton.shortPress();

            intakeButton.updateButton(utilityGamepad.dpad_up);
            intakeButton.shortPress();

            planeButton.updateButton(drivingGamepad.a);
            planeButton.longPress();

            robotCentricDrive.setReverse(reverseButton.getShortToggle());

            robot.servos.arm(utilityGamepad.left_bumper && utilityGamepad.right_bumper ? 0 : utilityGamepad.right_bumper ? -1 : utilityGamepad.left_bumper ? 1 : 0);
            double clawPosition = utilityGamepad.b && utilityGamepad.a ? 0 : utilityGamepad.a ? 1 : utilityGamepad.b ? -1 : 0;
            robot.servos.claw(clawPosition);
            robot.servos.intake(intakeButton.getShortToggle());

            if (!driveModeButton.getLongToggle()) {
                robotCentricDrive.run();
                robotCentricDrive.telemetry(telemetry);
            } else {
                fieldCentricDrive.run();
                fieldCentricDrive.telemetry(telemetry);
            }

            liftDriver.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
            telemetry.update();
        }
    }
}
