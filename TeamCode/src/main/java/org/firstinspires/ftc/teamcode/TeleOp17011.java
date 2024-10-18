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

import static org.firstinspires.ftc.teamcode.Constants.climberTickPerIn;
import static org.firstinspires.ftc.teamcode.Constants.pivotTickPerDegree;
import static org.firstinspires.ftc.teamcode.Constants.slideTickPerIn;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.roadRunnerActions.Intake;

import java.sql.RowId;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp17011", group="Linear OpMode")
@Config

public class TeleOp17011 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo Intake_Claw;
    private Servo Intake_Spin;
    private Servo Intake_Pivot;
    private DcMotor Linear_Slide;
    private DcMotor Slide_Pivot;
    private DcMotor Right_Climber;
    private DcMotor Left_Climber;
    private Servo Right_Hook;
    private Servo Left_Hook;
    public static double NEW_P = 10;
    public static double NEW_I = 3;
    public static double NEW_D = 0;

    FtcDashboard dash;

    public void Slide_Extend_High_Basket() {
        if (gamepad2.x) {
            Linear_Slide.setPower(0.7);
            Linear_Slide.setTargetPosition((int)(31 * slideTickPerIn));
        }
    }
    public void Slide_Extend_Spec_High() {
        if (gamepad2.a) {
            Linear_Slide.setPower(0.7);
            Linear_Slide.setTargetPosition((int)(20 * slideTickPerIn));
        }
    }
    public void Slide_Pivot_Start() {
        if (gamepad2.b) {
            Slide_Pivot.setPower(0.7);
            Slide_Pivot.setTargetPosition(0);
        }
    }
    public void Slide_Pivot_Expand() {
        if (gamepad2.y) {
            Slide_Pivot.setPower(0.7);
            Slide_Pivot.setTargetPosition((int)(30 * pivotTickPerDegree));
        }
    }
    public void Climbers_Up() {
        if  (gamepad2.right_trigger > .01) {
            Right_Climber.setPower(0.7);
            Right_Climber.setTargetPosition((int)(5*climberTickPerIn));
            Left_Climber.setPower(0.7);
            Left_Climber.setTargetPosition((int)(5*climberTickPerIn));
        }
    }
    public void Climbers_Down() {
        if  (gamepad2.left_trigger > .01) {
            Right_Climber.setPower(0.7);
            Right_Climber.setTargetPosition(0);
            Left_Climber.setPower(0.7);
            Left_Climber.setTargetPosition(0);
        }
    }
    public void Hooks_Up() {
        if(gamepad2.dpad_up) {
            Left_Hook.setPosition(1);
            Right_Hook.setPosition(1);
        }
    }
    public void Hooks_Down() {
        if(gamepad2.dpad_down) {
            Left_Hook.setPosition(1);
            Right_Hook.setPosition(1);
        }
    }
    public void Claw_Close() {
        if (gamepad1.y) {
            Intake_Claw.setPosition((int)(0));
        }
    }
    public void Claw_Open() {
        if (gamepad1.b) {
            Intake_Claw.setPosition((int)(1));
        }
    }
    public void Spin_Start() {
        if (gamepad1.x) {
            Intake_Spin.setPosition((int)(0));
        }
    }
    public void Spin_Full() {
        if (gamepad1.a)
            Intake_Spin.setPosition((int)(1));
    }
    public void Intake_Pivot_Start() {
        if (gamepad1.right_trigger > 0.01) {
            Intake_Pivot.setPosition(0);
        }
    }
    public void Intake_Pivot_Expand() {
        if(gamepad1.left_trigger > 0.01) {
            Intake_Pivot.setPosition(1);
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

        Linear_Slide = hardwareMap.get(DcMotor.class, "lift");
        Linear_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linear_Slide.setDirection(DcMotor.Direction.FORWARD);

        Slide_Pivot = hardwareMap.get(DcMotor.class, "pivot");
        Slide_Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        Right_Climber = hardwareMap.get(DcMotorEx.class, "climberR");
//        Right_Climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        Left_Climber = hardwareMap.get(DcMotorEx.class, "climberL");
//        Left_Climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        Left_Hook = hardwareMap.get(Servo.class, "hookL");
//        Right_Hook = hardwareMap.get(Servo.class, "hookR");

        Intake_Claw = hardwareMap.get(Servo.class, "claw");

        Intake_Spin = hardwareMap.get(Servo.class, "spinX");

        Intake_Pivot = hardwareMap.get(Servo.class, "pivotY");


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

            Slide_Extend_High_Basket();
            Slide_Extend_Spec_High();
            Slide_Pivot_Expand();
            Slide_Pivot_Start();
            Intake_Pivot_Expand();
            Intake_Pivot_Start();
            Claw_Close();
            Claw_Open();
            Spin_Full();
            Spin_Start();

//            if (max > 1.0) {
//                leftFrontPower  /= max;
//                rightFrontPower /= max;
//                leftBackPower   /= max;
////                rightBackPower  /= max;
//            }
//
//            if (gamepad1.x) {
//                pidNew.p = NEW_P;
//                pidNew.i = NEW_I;
//                pidNew.d = NEW_D;
//                rightBackDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//            }
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

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
