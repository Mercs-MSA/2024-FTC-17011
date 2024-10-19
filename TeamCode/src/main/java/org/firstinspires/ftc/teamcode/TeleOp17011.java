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

import static org.firstinspires.ftc.teamcode.Constants.pivotTickPerDegree;
import static org.firstinspires.ftc.teamcode.Constants.slideTickPerIn;

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
    private Servo intakePivot;
    private Servo specimenIntake;
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    private DcMotorEx pivot;
    private Servo Right_Hook;
    private Servo Left_Hook;
    public static double NEW_P = 10;
    public static double NEW_I = 3;
    public static double NEW_D = 0;

    FtcDashboard dash;

    public void configureScoringMechanism() {
        //Lift Up
        if (gamepad2.x) {
//            slidePIDFNew = new PIDFCoefficients(0.00111, slidePIDFOrig.i, slidePIDFOrig.d, slidePIDFOrig.f);
//            leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, slidePIDFNew);
//            rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, slidePIDFNew);
            leftSlide.setPower(.7);
            rightSlide.setPower(.7);
            leftSlide.setTargetPosition((int) (31 * slideTickPerIn));
            rightSlide.setTargetPosition((int) (31 * slideTickPerIn));
        }

        //Lift Down
        if (gamepad2.a) {
            leftSlide.setPower(.7);
            rightSlide.setPower(.7);
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
        }

        //Pivot Up
        if (gamepad2.y) {
            pivot.setPower(0.7);
            pivot.setTargetPosition(0);
        }

        //Pivot Down
        if (gamepad2.b) {
            pivot.setPower(0.7);
            pivot.setTargetPosition((int) (90 * pivotTickPerDegree));
        }
    }

        
    public void intakeCode() {
        if (gamepad1.y) {
            intake.setPosition((int) (0));
        }

        if (gamepad1.b) {
            intake.setPosition((int) (1));
        }

        if (gamepad1.x) {
            intakePivot.setPosition((int) (0));
        }

        if (gamepad1.a) {
            intakePivot.setPosition((int) (1));
        }
        
        if (gamepad1.right_trigger > 0.01) {
            specimenIntake.setPosition(0);
        }
        
        if (gamepad1.left_trigger > 0.01) {
            specimenIntake.setPosition(1);
        }
    }



    @Override
    public void runOpMode() {
        MecanumDrive drive1 = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

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

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        PIDFCoefficients slidePIDFOrig = leftSlide.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients slidePIDFNew = new PIDFCoefficients(0,0,0,0);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setTargetPosition(0);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setTargetPosition(0);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(Servo.class, "intake");

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        specimenIntake = hardwareMap.get(Servo.class, "specimenIntake");


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

            configureScoringMechanism();
            intakeCode();

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
           // rightBackDrive.setVelocity(rightBackPower*2500);
            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            drive1.updatePoseEstimate();
//            telemetry.addData("X: ", drive1.poseOTOS.position.x);
//            telemetry.addData("Y: ", drive1.poseOTOS.position.y);
//            telemetry.addData("Theta: ", Math.toDegrees(drive1.poseOTOS.heading.toDouble()));
            telemetry.addData("X-pod: ", drive1.pose.position.x);
            telemetry.addData("Y-pod: ", drive1.pose.position.y);
            telemetry.addData("Heading: ", drive1.pose.heading);
//            telemetry.addData("Current velocity:", rightBackDrive.getVelocity());
//            telemetry.addData("Current power:", (rightBackDrive.getPower()*2500));
//            telemetry.addData("PID Values:", pidNew);
//            telemetry.addData("Old pidOrig:", pidOrig2);
            telemetry.update();
        }
    }
}
