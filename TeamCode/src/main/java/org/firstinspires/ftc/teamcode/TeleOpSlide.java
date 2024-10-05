package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
//Neverest Motor Slide
public class TeleOpSlide extends LinearOpMode {
    public DcMotor slide = null;

    public double slideTicksPerInches = 600/6.25;
    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setPower(0.5);
        double distanceInches = 0;
        waitForStart();
        while (opModeIsActive()) {
            slide.setTargetPosition((int)(distanceInches * slideTicksPerInches));
            if (gamepad1.right_bumper && slide.getCurrentPosition() <= distanceInches * slideTicksPerInches) {
                distanceInches += .1;
            }
            else if(gamepad1.a) {
                slide.setTargetPosition(0);
            }
            else if (gamepad1.b  && slide.getCurrentPosition() >= distanceInches * slideTicksPerInches) {
                distanceInches -= .1;
            }
            telemetry.addData("Encoder Count: ", slide.getCurrentPosition());
            telemetry.update();
        }
    }


}
