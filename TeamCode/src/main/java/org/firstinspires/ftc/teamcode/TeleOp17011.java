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
import static org.firstinspires.ftc.teamcode.Constants.highBasketPos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecScorePos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecimenPos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecimenState;
import static org.firstinspires.ftc.teamcode.Constants.intakeHoldPos;
import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotGrabPos;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinDefault;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinLeft;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinRight;
import static org.firstinspires.ftc.teamcode.Constants.lowBasketPos;
import static org.firstinspires.ftc.teamcode.Constants.normalSpeed;
import static org.firstinspires.ftc.teamcode.Constants.pivotDownPos;
import static org.firstinspires.ftc.teamcode.Constants.pivotUpPos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;

import static org.firstinspires.ftc.teamcode.Constants.pivotTickPerDegree;
import static org.firstinspires.ftc.teamcode.Constants.slowSpeed;
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
    public IMU imu = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public Intakes intakes;
    public Slides slides;
    public Pivot pivot;
    public static double NEW_P = 18;
    public static double NEW_I = 1;
    public static double NEW_D = 0.4;
    public static double NEW_F = 1;


    public boolean pivotBool = false;
    public boolean specimenBool = true;
    public boolean slideReadyBool = false;
    public int mechanismState = defaultState;

    public boolean intakeControl = true;
    public boolean intPivotControl = true;

    public double speedMultiplier;
    FtcDashboard dash;

    private void initializeImu() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void initializeMotors() throws InterruptedException {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakes = new Intakes(hardwareMap);

        slides = new Slides(hardwareMap);

        pivot = new Pivot(hardwareMap, NEW_P, NEW_I, NEW_D, NEW_F);
    }
    public void scoringCode() {
        //Lift High Basket
        if (gamepad2.dpad_up) {
            if (!pivotBool) {
                slides.slideSetPos(highBasketPos);
            } else if (pivotBool) {
                slides.slideSetPos(lowBasketPos);
            }
        } else if (gamepad2.dpad_left) {
            slides.slideSetPos(60); //Specimen Grab
        }

        //Lift High Specimen
        if (gamepad2.left_bumper) {
            if (!pivotBool) {
                intakes.pivotSetPos(intakePivotGrabPos);
                slides.slideSetPos(highSpecimenPos);
            } else if (pivotBool) {
                slides.slideSetPos(lowBasketPos);
            }
        } else if (gamepad2.right_bumper) {
            if (!pivotBool) {
                intakes.specSetPos(specimenHoldPos);
                slides.slideSetPos(highSpecScorePos);
            } else if (pivotBool) {
                slides.slideSetPos(lowBasketPos);
            }
        }

        if (gamepad2.dpad_right) {
            slides.slideSetPos(lowBasketPos);
        }

        //Lift Down
        if (gamepad2.dpad_down) {
            slides.slideSetPos(0);
            pivotBool = true;
            pivot.pivotSetPos(pivotDownPos);
        }

        if (gamepad2.x) {
            pivotBool = false;
            pivot.pivotSetPos(pivotUpPos);
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
        if (gamepad1.right_bumper) {
            intakes.clawSetPos(intakeScorePos);
            intakes.specSetPos(specimenScorePos);
        } else if (gamepad1.left_bumper) {
            intakes.clawSetPos(intakeHoldPos);
            intakes.specSetPos(specimenHoldPos);
        }

        if (gamepad2.right_trigger > .3) {
            intakes.spinSetPos(intakeSpinRight);
        } else if (gamepad2.left_trigger > .3) {
            intakes.spinSetPos(intakeSpinLeft);
        } else {
            intakes.spinSetPos(defaultState);
        }

        if (gamepad2.y) {
            intakes.pivotSetPos(intakePivotScorePos);
        } else if (gamepad2.a) {
            intakes.pivotSetPos(intakePivotGrabPos);
        } else if (gamepad2.b) {
            intakes.pivotSetPos(.3);
        }
    }
    private void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Calculate the current angle of the robot (yaw) relative to the field.
        double robotAngle = -Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); // Modify this to get the actual robot angle.

        // Calculate the field-centric components of movement.
        double fieldX = x * Math.cos(robotAngle) - y * Math.sin(robotAngle);
        double fieldY = x * Math.sin(robotAngle) + y * Math.cos(robotAngle);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(rx), 1);
        double frontLeftPower = (fieldY + fieldX + rx) / denominator;
        double backLeftPower = (fieldY - fieldX + rx) / denominator;
        double frontRightPower = (fieldY - fieldX - rx) / denominator;
        double backRightPower = (fieldY + fieldX - rx) / denominator;

        // Set the motor powers.
        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public void mechanumDrive() {
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

        leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
        rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
        leftBackDrive.setPower(leftBackPower * speedMultiplier);
        rightBackDrive.setPower(rightBackPower * speedMultiplier);
    }



    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dash = FtcDashboard.getInstance();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        initializeMotors();

        initializeImu();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            scoringCode();
            intakeCode();
//            fieldCentricDrive();
            mechanumDrive();

            if (pivot.getPos() <= (70) && pivotBool == true) {
                pivot.setPow(0);
            } else if (pivotBool == false) {
                pivot.setPow(1);
                if (pivot.getPos() > 420) {
                    pivot.setPow(0);
                }
            } else if (pivotBool) {
                pivot.setPow(.3);
            }

//            if (slideReadyBool && mechanismState == highBasketState) {
//                leftSlide.setTargetPosition((int) (31 * slideTickPerIn));
//                rightSlide.setTargetPosition((int) (31 * slideTickPerIn));
//            } else if (slideReadyBool && mechanismState == defaultState) {
//                leftSlide.setTargetPosition(0);
//                rightSlide.setTargetPosition(0);
//            }
            if (gamepad1.right_trigger > .5) {
                speedMultiplier = slowSpeed;
            } else {
                speedMultiplier = normalSpeed;
            }

            // Send calculated power to wheels
            telemetry.addData("Pivot: ", pivot.getPos());
            telemetry.addData("left slide: ", slides.getLeftPos());
            telemetry.addData("right slide: ", slides.getRightPos());
//            telemetry.addData("Current velocity:", rightBackDrive.getVelocity());
//            telemetry.addData("Current power:", (rightBackDrive.getPower()*2500));
//            telemetry.addData("PID Values:", pidNew);
//            telemetry.addData("Old pidOrig:", pidOrig2);
            telemetry.update();
        }
    }
}
