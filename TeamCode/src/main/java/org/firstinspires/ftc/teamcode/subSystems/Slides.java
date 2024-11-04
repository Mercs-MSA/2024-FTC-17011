package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    public Slides(HardwareMap hardwareMap) throws InterruptedException {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");;
//        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        leftSlide.setPower(1);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setTargetPosition(0);
        rightSlide.setPower(1);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slideSetPos(int pos) {
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
    }

    public int getRightPos() {
        return rightSlide.getCurrentPosition();
    }

    public int getLeftPos() {
        return leftSlide.getCurrentPosition();
    }
}
