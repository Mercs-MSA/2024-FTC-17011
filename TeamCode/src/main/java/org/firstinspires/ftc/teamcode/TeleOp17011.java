/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.defaultState;
import static org.firstinspires.ftc.teamcode.Constants.highSpecimenPos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecimenState;
import static org.firstinspires.ftc.teamcode.Constants.intakeHoldPos;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotGrabPos;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinDefault;
import static org.firstinspires.ftc.teamcode.Constants.pivotDownPos;
import static org.firstinspires.ftc.teamcode.Constants.pivotUpPos;
import static org.firstinspires.ftc.teamcode.Constants.pivotTickPerDegree;
import static org.firstinspires.ftc.teamcode.Constants.slideTickPerIn;
import static org.firstinspires.ftc.teamcode.Constants.specimenHoldPos;
import static org.firstinspires.ftc.teamcode.Constants.specimenScorePos;

import static org.firstinspires.ftc.teamcode.Constants.highBasketState;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;


@TeleOp(name="TeleOp17011", group="Linear OpMode")
@Config

public class TeleOp17011 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo intake;
    private Servo intakeSpin;
    private Servo intakePivot;
    private Servo specimenIntake;
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    private DcMotorEx pivot;
    private Servo Right_Hook;
    private Servo Left_Hook;
    public static double NEW_P = 18;
    public static double NEW_I = 1;
    public static double NEW_D = 0.2;
    public static double NEW_F = 0.2;
    public static double intakePos = 1;
    public static double intakeSpinPos = 1;
    public static double intakePivotPos = 1;
    public static double specimenPos = 1;

    public int leftSlidePower = 0;
    public int rightSlidePower = 0;

    public boolean pivotBool = false;
    public boolean specimenBool = true;
    public boolean slideReadyBool = false;
    public int mechanismState = defaultState;

    public boolean intakeControl = true;
    public boolean intPivotControl = true;
    FtcDashboard dash;


    public void scoringCode() {
        //Lift High Basket
        if (gamepad2.y) {
            leftSlide.setTargetPosition((int) (33 * slideTickPerIn));
            rightSlide.setTargetPosition((int) (33 * slideTickPerIn));
        }

        //Lift High Specimen
        if (gamepad2.dpad_up) {
            leftSlide.setTargetPosition((int) (20 * slideTickPerIn));
            rightSlide.setTargetPosition((int) (20 * slideTickPerIn));
        }

        //Lift Down
        if (gamepad2.a) {
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
        }

        //Pivot
        if (gamepad2.b) {
            boolean temp = true;
            if (temp) {
                pivotBool = false;
                pivot.setTargetPosition(pivotUpPos);
            } else if (!temp) {
                pivotBool = true;
//            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pivot.setTargetPosition(pivotDownPos);
            }
        }

        //Score High Basket
//        if (gamepad2.dpad_up) {
//            mechanismState = highBasketState;
//            pivotBool = false;
//            slideReadyBool = false;
//            pivot.setTargetPosition(pivotUpPos);
//            if (pivot.getCurrentPosition() > pivotUpPos) {
//                slideReadyBool = true;
//            }
//            intakeSpin.setPosition(intakeSpinDefault);
//            intakePivot.setPosition(intakePivotScorePos);
//            if (intakePivot.getPosition() == intakePivotScorePos) {
//                intake.setPosition(intakeScorePos);
//            }
//            //Default Pos
//        } else if (gamepad2.dpad_down) {
//            mechanismState = defaultState;
//            intakeSpin.setPosition(intakeSpinDefault);
//            pivotBool = true;
//            pivot.setTargetPosition(pivotDownPos);
//            intake.setPosition(intakeScorePos);
//            intakePivot.setPosition(intakePivotGrabPos);
//        } else if (gamepad2.y) { //High Specimen
//            if (specimenBool) {
//                mechanismState = highSpecimenState;
//                leftSlide.setTargetPosition((int)(20 * slideTickPerIn));
//                rightSlide.setTargetPosition((int)(20 * slideTickPerIn));
//                pivotBool = false;
//                pivot.setTargetPosition(pivotUpPos);
//                intakeSpin.setPosition(intakeSpinDefault);
//                specimenIntake.setPosition(specimenHoldPos);
//                specimenBool = true;
//            } else if (specimenBool == false) {
//                leftSlide.setTargetPosition((int)(19 * slideTickPerIn));
//                rightSlide.setTargetPosition((int)(19 * slideTickPerIn));
//                specimenIntake.setPosition(specimenScorePos);
//                specimenBool = false;
//            }
//        }
//        leftSlide.setTargetPosition(leftSlidePower);
//        rightSlide.setTargetPosition(rightSlidePower);
    }



    public void intakeCode() {
        if (gamepad1.b) {
            if (intakeControl && intake.getPosition() == intakeHoldPos) {
                intake.setPosition(intakeScorePos);
                intakeControl = false;
            } else if (!intakeControl && intake.getPosition() == intakeScorePos) {
                intake.setPosition(intakeHoldPos);
                intakeControl = true;
            }
        }

        if (gamepad1.x) {
            intakeSpin.setPosition(intakeSpinDefault);
        }

        if (gamepad1.y) {
            if (intPivotControl && intakePivot.getPosition() == intakePivotScorePos) {
                intakePivot.setPosition(intakePivotGrabPos);
                intPivotControl = false;
            } else if (!intPivotControl && intakePivot.getPosition() == intakePivotGrabPos) {
                intakePivot.setPosition(intakePivotScorePos);
                intPivotControl = true;
            }
        }
    }




    @Override
    public void runOpMode() {
        MecanumDrive drive1 = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        FtcDashboard dash = FtcDashboard.getInstance();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        intake = hardwareMap.get(Servo.class, "intake");

        intakeSpin = hardwareMap.get(Servo.class, "intakeSpin");

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        specimenIntake = hardwareMap.get(Servo.class, "specimenIntake");

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setTargetPositionTolerance(3);
        PIDFCoefficients pivotPIDFNew = new PIDFCoefficients(NEW_P,NEW_I,NEW_D,NEW_F);
        pivot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pivotPIDFNew);
        pivot.setTargetPosition(0);
        pivot.setPower(1);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");;
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
//        PIDFCoefficients slidePIDFOrig = leftSlide.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients slidePIDFNew = new PIDFCoefficients(0,0,0,0);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        leftSlide.setPower(.7);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setTargetPosition(0);
        rightSlide.setPower(.7);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setPosition(intakeHoldPos);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
//            dash = FtcDashboard.getInstance();
//            telemetry = dash.getTelemetry();

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            leftSlidePower = (int)(-gamepad2.left_stick_y * 20);
            rightSlidePower = (int)(-gamepad1.left_stick_y * 20);

            scoringCode();
            intakeCode();

            if (pivot.getCurrentPosition() <= (70) && pivotBool == true) {
                pivot.setPower(-0.05);
            } else if (pivotBool == false) {
                pivot.setPower(1);
                if (pivot.getCurrentPosition() > 390) {
                    pivot.setTargetPosition(pivotUpPos + 30);
                }
            } else if (pivotBool) {
                pivot.setPower(.6);
            }

//            if (slideReadyBool && mechanismState == highBasketState) {
//                leftSlide.setTargetPosition((int) (31 * slideTickPerIn));
//                rightSlide.setTargetPosition((int) (31 * slideTickPerIn));
//            } else if (slideReadyBool && mechanismState == defaultState) {
//                leftSlide.setTargetPosition(0);
//                rightSlide.setTargetPosition(0);
//            }
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            drive1.updatePoseEstimate();
//            telemetry.addData("X: ", drive1.poseOTOS.position.x);
//            telemetry.addData("Y: ", drive1.poseOTOS.position.y);
//            telemetry.addData("Theta: ", Math.toDegrees(drive1.poseOTOS.heading.toDouble()));
            telemetry.addData("X-pod: ", drive1.pose.position.x);
            telemetry.addData("Y-pod: ", drive1.pose.position.y);
            telemetry.addData("Heading: ", drive1.pose.heading);
            telemetry.addData("Pivot: ", pivot.getCurrentPosition());
//            telemetry.addData("Current velocity:", rightBackDrive.getVelocity());
//            telemetry.addData("Current power:", (rightBackDrive.getPower()*2500));
//            telemetry.addData("PID Values:", pidNew);
//            telemetry.addData("Old pidOrig:", pidOrig2);
            telemetry.update();
        }
    }
}
