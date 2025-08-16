package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTest", group="Test")
public class MotorTest extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        // Initial directions (you can change them to test)
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Motor Test Initialized");
        telemetry.addLine("Press buttons to test each motor:");
        telemetry.addLine("Y = Left Front, X = Right Front, B = Left Back, A = Right Back");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Stop all motors by default
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Test individual motors
            if (gamepad1.y) {
                leftFrontDrive.setPower(0.5);
            }
            if (gamepad1.x) {
                rightFrontDrive.setPower(0.5);
            }
            if (gamepad1.b) {
                leftBackDrive.setPower(0.5);
            }
            if (gamepad1.a) {
                rightBackDrive.setPower(0.5);
            }

            telemetry.addData("Left Front Power", leftFrontDrive.getPower());
            telemetry.addData("Right Front Power", rightFrontDrive.getPower());
            telemetry.addData("Left Back Power", leftBackDrive.getPower());
            telemetry.addData("Right Back Power", rightBackDrive.getPower());
            telemetry.update();
        }
    }
}
