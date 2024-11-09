package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DriveTester extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); //V1 - REVERSE
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); //V1 - REVERSE
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); //V1 - FORWARD
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE); //V1 - FORWARD

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                leftFrontDrive.setPower(-gamepad1.left_stick_y);
            }
            if (gamepad1.right_bumper) {
                rightFrontDrive.setPower(-gamepad1.left_stick_y);
            }
            if (gamepad1.left_trigger > .3) {
                leftBackDrive.setPower(-gamepad1.left_stick_y);
            }
            if (gamepad1.right_trigger > .3) {
                rightBackDrive.setPower(-gamepad1.left_stick_y);
            }

            telemetry.addData("Left Front Power: ", leftFrontDrive.getPower());
            telemetry.addData("Right Front Power: ", rightFrontDrive.getPower());
            telemetry.addData("Left Back Power: ", leftBackDrive.getPower());
            telemetry.addData("Right Back Power: ", rightBackDrive.getPower());
            telemetry.update();
        }
    }
}
