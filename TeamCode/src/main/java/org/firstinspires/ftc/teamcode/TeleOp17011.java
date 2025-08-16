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

import static org.firstinspires.ftc.teamcode.Constants.climberClimbedPos;
import static org.firstinspires.ftc.teamcode.Constants.climberReadyPos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecimenPos;
import static org.firstinspires.ftc.teamcode.Constants.pivotClimbPos;
import static org.firstinspires.ftc.teamcode.Constants.slideClimbDownPos;
import static org.firstinspires.ftc.teamcode.Constants.slideClimbPos;
import static org.firstinspires.ftc.teamcode.Constants.extendPos;
import static org.firstinspires.ftc.teamcode.Constants.highBasketPos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeCollectPow;
//import static org.firstinspires.ftc.teamcode.Constants.intakeHoldPos;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private enum SAMPLECYCLE_STATE {
        PIVOT_DOWN,
        INTAKE_PIVOT,
        INTAKE_PIVOT_DOWN,
        LIFT_RETRACT,
    }
    public SAMPLECYCLE_STATE currentState = SAMPLECYCLE_STATE.PIVOT_DOWN;

//    public DcMotorEx leftPivot = null;
//    public DcMotorEx rightPivot = null;
    public boolean lift = true;
    public boolean pivotBool = true;
    public boolean initSlides = true;

    public boolean intPivotControl = true;
    public boolean highSpec = true;
    public double speedMultiplier;
    private boolean climbTrue = false;
    private boolean encoderTrue = true;

    public int intakePivotCount = 0;
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
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); //V1 - REVERSE //V2 - FORWARD
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); //V1 - REVERSE //V2 - FORWARD
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); //V1 - FORWARD //V2 - REVERSE
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE); //V1 - FORWARD //V2 - REVERSE

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakes = new Intakes(hardwareMap, telemetry);

        slides = new Slides(hardwareMap);

        pivot = new Pivot(hardwareMap, pivotP, pivotI, pivotD, pivotF);

    }

//    TODO: Button Mapping I want/Simpler Controls:
//    Left Trigger - Intake Spin Left
//    Right Trigger - Intake Spin Right
//    Both Triggers - Intake Spin Backwards
//    Right Bumper - Press once, makes Lift go up to High Specimen position, Press again, makes Lift go down slightly  to place specimen
//    Left Bumper - Toggle sequence for sample pickup, on the first press of the button it makes sure the pivot is down, then makes sure the intake is up, then sends the lift out, then when the lift is fully out, the intake pivots down, Then when I see theres a sample in the intake, I press the button again, the Intake pivots up, Lift retracts

//    Dpad UP - Brings the lift to the high basket position, makes sure that the intake is either pivoted in down or middle position, then when it sees the lift is in the highest position the intake pivots to the UP position
//    Dpad Left - Level 2 Climb Sequence
//    Dpad Right - Level 3 Climb Sequence
//    Dpad Down - Reset button, brings the elevator down, pivot down, and makes sure the intake is in MIDDLE while the pivot goes down
//    PS5 Triangle (Xbox Y) - Intake Pivot UP (Precautions if the sequences above dont work)
//    PS5 Circle (Xbox B) - Intake Pivot MIDDLE (Precautions if the sequences above dont work)
//    PS5 X (Xbox A) - Intake Pivot DOWN (Precautions if the sequences above dont work)
//    PS5 Square (Xbox X) - Toggle Pivot Up
//    Joystick Left - Fine tuning on Elevator
//    Joystick Right - Nothing (Could be used for either manual intake or manual pivot control)
//    Alternate Mode Using the Middle PS5 Button (reset mode for is auton sets the robot in a weird position):
//    Joystick Left - Brings Lift down
//    Joystick Right - Brings Pivot Down
//    When I press the alternate mode button again it will reset all the encoder values and the robot should work normally

    public void gamepadTwo_Main() {
        if (gamepad2.dpad_up && !pivotBool) {
            slides.slideSetPos(highBasketPos);
            intakes.pivotSetPos(intakePivotMidPos);
//            lift = false;
        } else if (gamepad2.dpad_up && pivotBool) {
            slides.slideSetPos(extendPos);
//            intakes.pivotSetPos(intakePivotMidPos);
        }

//        if (!lift && (slides.getLeftPos() > 2200)) {
//            intakes.pivotSetPos(intakePivotScorePos);
//            lift = true;
//        }

        //Nandan
        if (gamepad2.dpad_down) { // DEF PROBLEM WITH THE PIVOTBOOL
            slides.slideSetPos(0);
            pivotBool = true;
            pivot.pivotSetPos(pivotDownPos);
            intakes.pivotSetPos(intakePivotMidPos);
        } else if (Math.abs(gamepad2.left_stick_y) > .1) { // Lift up (needs joystick left fine tuning)
            slides.slideSetPos(slides.getLeftPos() + (int) (-gamepad2.left_stick_y * 25));
        }

        if (gamepad2.y) { // UP POS
            intPivotControl = false;
            intakes.pivotSetPos(intakePivotScorePos);
        }
        else if (gamepad2.a) { // DOWN POS
            intPivotControl = true;
            intakes.pivotSetPos(intakePivotGrabPos);
        }
        if (gamepad2.circle) { // MID POS
            intPivotControl = false;
            intakes.pivotSetPos(intakePivotMidPos);
        }
        if (gamepad2.square) { // PIVOT UP
            pivotBool = false;
            pivot.pivotSetPos(pivotUpPos);
        }
        if (gamepad2.touchpad) { // RESET POS FOR AUTO
            if (encoderTrue) {
                encoderTrue = false;
                pivotBool = true;
                pivot.normalMode();
                intakes.pivotSetPos(intakePivotMidPos);
                sleep(200);
            } else {
                encoderTrue = true;
                pivot.encoderMode(pivotP, pivotI, pivotD, pivotF);
                sleep(200);
            }
        }
        if (gamepad2.right_stick_y > .1 && !encoderTrue)
            pivot.setPow(-1);
        if (gamepad2.left_stick_y > .1  && !encoderTrue)
            slides.slideSetPos(-1500);

        //Joshua
        if (gamepad2.right_trigger > .3 && gamepad2.left_trigger < .3) {
            intakes.spinSetPos(intakeSpinRight);
        } else if (gamepad2.left_trigger > .3 && gamepad2.right_trigger < .3) {
            intakes.spinSetPos(intakeSpinLeft);
        } else if (gamepad2.right_trigger > .3 && gamepad2.left_trigger > .3) {
            intakes.spinSetPos(intakeSpinBack);
        }else {
            intakes.spinSetPos(intakeSpinDefault);
        }

        // CLIMB STAGE 2
        if (gamepad2.dpad_left && !climbTrue) {
            slides.slideSetPos(0);
        }

//        if (climbTrue && slides.getLeftPos() < slideClimbDownPos + 15) {
//            climber.setClimbers(climberClimbedPos);
//            sleep(300);
//            slides.slideSetPos(slideClimbPos + 100);
//            sleep(300);
//            pivot.pivotSetPos(pivotUpPos);
//        }

        // CLIMB STAGE 3
//        if (gamepad2.dpad_right && climbTrue) {
//            slides.slideSetPos(slideClimbPos);
//        } else if (climbTrue && gamepad2.dpad_right && slides.getLeftPos() > slideClimbPos - 20) {
//            pivot.pivotSetPos(pivotClimbPos);
//            sleep(300);
//            climber.setClimbers(climberReadyPos);
//            slides.slideSetPos(slideClimbDownPos);
//        }
    }

    public void gamepadTwo_Extras() {
        if (gamepad2.right_bumper && highSpec && slides.getLeftPos() < (highSpecScorePos + 20)) {
            slides.slideSetPos(highSpecimenPos);
            highSpec = false;
        } else if(gamepad2.right_bumper && !highSpec && slides.getLeftPos() > highSpecScorePos) {
            slides.slideSetPos(highSpecScorePos);
            highSpec = true;
        }

        if (gamepad2.left_bumper && (currentState == SAMPLECYCLE_STATE.PIVOT_DOWN)) {
            if (!pivotBool) {
                pivot.pivotSetPos(pivotDownPos);
                pivotBool = true;
            }
            intakes.pivotSetPos(intakePivotMidPos);
//            currentState = SAMPLECYCLE_STATE.INTAKE_PIVOT;
        } else if(gamepad2.left_bumper && (currentState == SAMPLECYCLE_STATE.INTAKE_PIVOT)) {
            slides.slideSetPos(extendPos);
//            currentState = SAMPLECYCLE_STATE.INTAKE_PIVOT_DOWN;
//        } else if(gamepad2.left_bumper && currentState == SAMPLECYCLE_STATE.INTAKE_PIVOT_DOWN && slides.getLeftPos() > extendPos - 40) {
            sleep(400);
            intakes.pivotSetPos(intakePivotGrabPos);
//            currentState = SAMPLECYCLE_STATE.LIFT_RETRACT;
        } else if(gamepad2.left_bumper && (currentState == SAMPLECYCLE_STATE.LIFT_RETRACT) && (slides.getLeftPos() > extendPos - 20)) {
            intakes.pivotSetPos(intakePivotMidPos);
            slides.slideSetPos(0);
            sleep(200);
//            currentState = SAMPLECYCLE_STATE.PIVOT_DOWN;
        }

        if (slides.getLeftPos() < 20 && pivot.getLeftPos() > 20) {
            currentState = SAMPLECYCLE_STATE.PIVOT_DOWN;
        } else if (slides.getLeftPos() < 20 && pivot.getLeftPos() < 20) {
            currentState = SAMPLECYCLE_STATE.INTAKE_PIVOT;
        } else if (slides.getLeftPos() > 20 && pivot.getLeftPos() < 20 && slides.getLeftPos() > extendPos - 20) {
            currentState = SAMPLECYCLE_STATE.LIFT_RETRACT;
        }
        //Nandan

        //Joshua

    }

    public void gamepadOne() {
        //Namish

        //Nandan

        //Joshua
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
        double axial   = -gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_y;
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

            //scoringCode();
            //intakeCode();
//            fieldCentricDrive();
            gamepadTwo_Main();
            gamepadTwo_Extras();
            gamepadOne();
            mechanumDrive();

            if (encoderTrue) {
                if ((pivot.getLeftPos() <= (20)) && pivotBool && !gamepad2.touchpad) {
                    pivot.setPow(0);
                    pivot.setMotorsOff();
                } else if (!pivotBool) {
                    pivot.setPow(.8);
                    pivot.setMotorsOn();
                    if (pivot.getLeftPos() > (pivotUpPos - 10)) {
                        pivot.setPow(0);
                        pivot.setMotorsOff();
                    }
                } else if ((pivot.getLeftPos() > 30) && pivotBool) {
                    pivot.setPow(.5);
                    pivot.setMotorsOn();
                    intakes.pivotSetPos(intakePivotScorePos);
                }

                if (pivotBool && (slides.getLeftPos() > extendPos) && (pivot.getLeftPos() < pivotDownPos)) {
                    slides.slideSetPos(extendPos);
                }
            }

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

            telemetry.addData("left slide: ", slides.getLeftPos());
            telemetry.addData("right slide: ", slides.getRightPos());
            if (pivotBool) {
                telemetry.addLine("Pivot is down/going down");
            } else {
                telemetry.addLine("Pivot is up/going up");
            }
            telemetry.addData("Pivot left pos: ", pivot.getLeftPos());
            telemetry.addData("Pivot right pos: ", pivot.getRightPos());
            telemetry.addData("Left Pivot power: ", pivot.getLeftPow());
            telemetry.addData("Right Pivot power: ", pivot.getRightPow());
            telemetry.addData("Left Pivot Current: ", pivot.getLeftCurrent());
            telemetry.addData("Right Pivot Current: ", pivot.getRightCurrent());
            telemetry.addData("Collection Enum: ", currentState);
            telemetry.addData("Intake pivot loop pos: ", intakes.pivotGetPos());
//            telemetry.addData("Left Slide power: ", slides.getLeftPow());
//            telemetry.addData("Left Slide power: ", slides.getRightPow());
            telemetry.update();
        }
    }
}
