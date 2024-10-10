package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class LiftTestCode extends LinearOpMode {
    public DcMotorEx lift = null;
    public DcMotorEx pivot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(.5);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //lift.setDirection(DcMotor.Direction.REVERSE);

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setPower(.5);
        pivot.setTargetPosition(0);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                lift.setTargetPosition(lift.getCurrentPosition() + (int)Constants.slideTickPerIn);
            }
            if (gamepad1.dpad_down) {
                lift.setTargetPosition(lift.getCurrentPosition() - (int)Constants.slideTickPerIn);
            }
            if (gamepad1.y) {
                lift.setTargetPosition((int)(32.5 * Constants.slideTickPerIn));
            } else if (gamepad1.a) {
                lift.setTargetPosition(0);
            }

            if (gamepad1.right_bumper) {
                pivot.setTargetPosition(0);
            } else if (gamepad1.left_bumper) {
                pivot.setTargetPosition(40);
            }
        }
    }
}
