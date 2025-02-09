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

import static org.firstinspires.ftc.teamcode.Constants.climbPos;
import static org.firstinspires.ftc.teamcode.Constants.extendPos;
import static org.firstinspires.ftc.teamcode.Constants.highBasketPos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecScorePos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecimenPos;
import static org.firstinspires.ftc.teamcode.Constants.intakeCollectPow;
//import static org.firstinspires.ftc.teamcode.Constants.intakeHoldPos;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotGrabPos;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotMidPos;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotScorePos;
//import static org.firstinspires.ftc.teamcode.Constants.intakeScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeScorePow;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinBack;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinDefault;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinLeft;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinRight;
import static org.firstinspires.ftc.teamcode.Constants.normalSpeed;
import static org.firstinspires.ftc.teamcode.Constants.pivotD;
import static org.firstinspires.ftc.teamcode.Constants.pivotDownPos;
import static org.firstinspires.ftc.teamcode.Constants.pivotF;
import static org.firstinspires.ftc.teamcode.Constants.pivotI;
import static org.firstinspires.ftc.teamcode.Constants.pivotP;
import static org.firstinspires.ftc.teamcode.Constants.pivotUpPos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;

import static org.firstinspires.ftc.teamcode.Constants.slowSpeed;
import static org.firstinspires.ftc.teamcode.Constants.specimenHoldPos;
import static org.firstinspires.ftc.teamcode.Constants.specimenScorePos;


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
//    public Pivot pivot;

    public DcMotorEx leftPivot = null;
    public DcMotorEx rightPivot = null;

    public boolean pivotBool = true;
    public boolean initSlides = true;

    public boolean intPivotControl = true;

    public double speedMultiplier;
    private boolean climbTrue = false;
    private boolean encoderTrue = true;
    FtcDashboard dash;
    private double operatorRightStick = 0;
    private double operatorLeftStick = 0;

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

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); //V1 - REVERSE //V2 - FORWARD
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); //V1 - REVERSE //V2 - FORWARD
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); //V1 - FORWARD //V2 - REVERSE
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE); //V1 - FORWARD //V2 - REVERSE

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakes = new Intakes(hardwareMap);

        slides = new Slides(hardwareMap);

//        pivot = new Pivot(hardwareMap, pivotP, pivotI, pivotD, pivotF);
//        pivot.normalMode();

        leftPivot = hardwareMap.get(DcMotorEx.class, "leftPivot");
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightPivot");

        leftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void scoringCode() {
        //GP2 dpad up = high basket slides
        //GP2 dpad down = everything down
        //GP2 dpad left = specimen grab prep slides position
        //GP2 dpad right = low basket/intake slides position
        //GP2 left bumper = high specimen pos
        //GP2 right bumper = high specimen score
        //GP2 x = pivot up
        //Lift High Basket
        if (gamepad2.dpad_up) {
            if (!pivotBool) {
                slides.slideSetPos(highBasketPos);
                intakes.pivotSetPos(intakePivotMidPos);
            } else if (pivotBool) {
                slides.slideSetPos(extendPos);
            }
        }
//        if (gamepad2.dpad_left && pivotBool) {
//            pivot.resetPos();
//        }

        //Lift High Specimen
        if (gamepad2.left_bumper) {
            if (!pivotBool) {
                intakes.pivotSetPos(intakePivotGrabPos);
                slides.slideSetPos(highSpecimenPos);
            } else if (pivotBool) {
                slides.slideSetPos(extendPos);
            }
        } else if (gamepad2.right_bumper) {
            if (!pivotBool) {
                intakes.specSetPos(specimenHoldPos);
                slides.slideSetPos(highSpecScorePos);
            } else if (pivotBool) {
                slides.slideSetPos(extendPos);
            }
        }

        if (gamepad2.dpad_right && !climbTrue) {
            slides.slideSetPos(climbPos);
            intakes.pivotSetPos(intakePivotMidPos);
            sleep(300);
            climbTrue = true;
        } else if (gamepad2.dpad_right && climbTrue){
            slides.slideSetPos(0);
            sleep(300);
            climbTrue = false;
        }

        //Lift Down
        if (gamepad2.dpad_down) {
            slides.slideSetPos(0);
        }
//        } else if (gamepad2.cross) {
//            pivotBool = true;
//            pivot.pivotSetPos(pivotDownPos);
//        }else if (gamepad2.triangle) { // Lift up
//            pivotBool = false;
//            pivot.pivotSetPos(pivotUpPos);
//        }

        if (gamepad2.touchpad) {
            if (encoderTrue) {
                encoderTrue = false;
                pivotBool = true;
//                pivot.normalMode();
                intakes.pivotSetPos(intakePivotMidPos);
                sleep(200);
            } else {
                encoderTrue = true;
//                pivot.encoderMode(pivotP, pivotI, pivotD, pivotF);
                sleep(200);
            }
        }

        if (gamepad2.right_stick_y > .1 && !encoderTrue) {
            slides.slideSetPos(-1500);
//            pivot.setPow(-1);
        }


//        if (Math.abs(gamepad2.left_stick_y) > .1) {
//            slides.slideSetPos(slides.getLeftPos() + ((int) (-gamepad2.left_stick_y * 70)));
            leftPivot.setPower(-gamepad2.left_stick_y);
            rightPivot.setPower(-gamepad2.left_stick_y);

        if (leftPivot.getPower() > 0)
            pivotBool = false;

        if (leftPivot.getPower() < 0) {
            pivotBool = true;
            leftPivot.setPower(leftPivot.getPower() * .5);
            rightPivot.setPower(rightPivot.getPower() * .5);
        }
//            pivot.pivotSetPos(pivot.getPos() + (int)(operatorLeftStick * 20));
//        }

//        if (gamepad1.triangle && pivotBool) {
//            pivotBool = false;
//            pivot.pivotSetPos(pivotClimbPos);
//        } else if (gamepad1.cross && !pivotBool) {
//            pivot.pivotSetPos(pivotDownPos);
//        }
    }



    public void intakeCode() {
        //GP1 right bumper = all open
        //GP1 left bumper = all close
        //GP2 right trigger = turn intake right
        //GP2 left trigger = turn intake left
        //GP2 y = intake pivot up
        //GP2 a = intake pivot down
        //GP2 b = intake pivot mid
        if (gamepad1.right_bumper) {
//            intakes.clawSetPos(intakeScorePos); V1
            intakes.setIntakePower(intakeScorePow);
            intakes.specSetPos(specimenScorePos);
        } else if (gamepad1.left_bumper) {
//            intakes.clawSetPos(intakeHoldPos); V1
            intakes.setIntakePower(intakeCollectPow);
            intakes.specSetPos(specimenHoldPos);
        } else {
            intakes.setIntakePower(0);
        }

        if (gamepad2.right_trigger > .3 && gamepad2.left_trigger < .3) {
            intakes.spinSetPos(intakeSpinRight);
        } else if (gamepad2.left_trigger > .3 && gamepad2.right_trigger < .3) {
            intakes.spinSetPos(intakeSpinLeft);
        } else if (gamepad2.right_trigger > .3 && gamepad2.left_trigger > .3) {
            intakes.spinSetPos(intakeSpinBack);
        }else {
            intakes.spinSetPos(intakeSpinDefault);
        }

        operatorRightStick = gamepad2.right_stick_y;
        if (Math.abs(gamepad2.right_stick_y) > .1) {
            intakes.pivotSetPos(intakes.pivotGetPos() + .1 * operatorRightStick);
//            intakes.pivotSetPow(operatorRightStick);
        }

//        if (gamepad2.y) {
//            intPivotControl = false;
//            intakes.pivotSetPos(intakePivotScorePos);
//        }
//        } else if (gamepad2.a) {
//            intPivotControl = true;
//            intakes.pivotSetPos(intakePivotGrabPos);
        if (gamepad2.circle) {
            intPivotControl = false;
//            intakes.pivotSetPos(intakePivotMidPos);
        }
    }
    private void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double leftBackPower = (rotY - rotX + rx) / denominator;
        double rightBackPower = (rotY + rotX - rx) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
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

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

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

//            if (encoderTrue) {
//                if (pivot.getPos() <= (20) && pivotBool && !gamepad2.touchpad) {
//                    pivot.setPow(0);
//                } else if (!pivotBool) {
//                    pivot.setPow(.8);
//                    if (pivot.getPos() > 240) {
//                        pivot.setPow(0);
//                    }
//                } else if (pivot.getPos() > 20 && pivotBool) {
//                    pivot.setPow(.3);
//                }
//
//                if (pivotBool && slides.getLeftPos() > extendPos && pivot.getPos() < pivotDownPos) {
//                    slides.slideSetPos(extendPos);
//                }
//            }

//            if (intPivotControl) {
//                intakes.pivotSetPos(intakePivotGrabPos);
//            }

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

//            telemetry.addData("Pivot: ", pivot.getPos());
            telemetry.addData("left slide: ", slides.getLeftPos());
            telemetry.addData("right slide: ", slides.getRightPos());
            if (pivotBool) {
                telemetry.addLine("Pivot is down/going down");
            } else {
                telemetry.addLine("Pivot is up/going up");
            }
            telemetry.addData("Left Pivot power: ", leftPivot.getPower());
            telemetry.addData("Right Pivot power: ", rightPivot.getPower());
            telemetry.addData("Left Pivot Current: ", leftPivot.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Pivot Current: ", rightPivot.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Left Slide power: ", slides.getLeftPow());
//            telemetry.addData("Left Slide power: ", slides.getRightPow());



            //            telemetry.addData("Current velocity:", rightBackDrive.getVelocity());
//            telemetry.addData("Current power:", (rightBackDrive.getPower()*2500));
//            telemetry.addData("PID Values:", pidNew);
//            telemetry.addData("Old pidOrig:", pidOrig2);
            telemetry.update();
        }
    }
}
