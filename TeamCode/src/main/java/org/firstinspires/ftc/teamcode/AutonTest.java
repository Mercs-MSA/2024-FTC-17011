package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.roadRunnerActions.AutonActions;


//This autonomous is for testing Road Runner
@TeleOp(name="Testing Road Runner", group="")
public class AutonTest extends LinearOpMode {
    MecanumDrive drive;
    private void initializeMotors() {
        drive.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    double StartPoseX = 0;
    double StartPoseY = -60;
    double StartPoseT = 90;
    @Override
    public void runOpMode() {
        Pose2d StartPose = new Pose2d(StartPoseX, StartPoseY, Math.toRadians(StartPoseT));
        //We are following this: https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/ while working on this. Setting starting pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);
        AutonActions lift = new AutonActions(hardwareMap);


        drive.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Action trajectoryAction1;

        Action trajectoryAction2;

        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(0, -32))
                .strafeTo(new Vector2d(-49, -32))
                .turn(Math.toRadians(180))
                .splineTo(new Vector2d(-53, -53), Math.toRadians(215))
                .build();
        waitForStart();

//        Actions.runBlocking(trajectoryAction1);
        Actions.runBlocking(lift.pivotDown());
    }
}
