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
//    private DcMotor pivot = null;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); //V1 - REVERSE //V2 - FORWARD
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); //V1 - REVERSE //V2 - FORWARD
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); //V1 - FORWARD //V2 - REVERSE
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD); //V1 - FORWARD //V2 - REVERSE

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        pivot = hardwareMap.get(DcMotor.class, "pivot");
//        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
//            leftFrontDrive.setPower(-(0.488888888889)*2);
//            rightBackDrive.setPower(-(0.488888888889)*2);
//            leftBackDrive.setPower(1);
//            rightFrontDrive.setPower(1);
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

//            if (gamepad2.triangle) {
//                pivot.setPower(1);
//            } else if (gamepad2.cross) {
//                pivot.setPower(-.5);
//            } else if (gamepad2.circle) {
//                pivot.setPower(0);
//            }

            telemetry.addData("Left Front Power: ", leftFrontDrive.getPower());
            telemetry.addData("Right Front Power: ", rightFrontDrive.getPower());
            telemetry.addData("Left Back Power: ", leftBackDrive.getPower());
            telemetry.addData("Right Back Power: ", rightBackDrive.getPower());
            telemetry.update();
        }
    }
}
