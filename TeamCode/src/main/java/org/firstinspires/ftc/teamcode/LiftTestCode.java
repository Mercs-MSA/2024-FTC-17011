package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class LiftTestCode extends LinearOpMode {
    public DcMotor lift = null;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(.5);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                lift.setTargetPosition(lift.getCurrentPosition() + 20);
            }
            if (gamepad1.dpad_down) {
                lift.setTargetPosition(lift.getCurrentPosition() - 20);
            }
            if (gamepad1.right_bumper) {
                lift.setTargetPosition(2000);
            } else if (gamepad1.x) {
                lift.setTargetPosition(0);
            }
        }
    }
}
