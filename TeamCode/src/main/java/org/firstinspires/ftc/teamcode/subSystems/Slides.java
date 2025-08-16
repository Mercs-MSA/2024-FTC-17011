package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    public Slides(HardwareMap hardwareMap) throws InterruptedException {

        leftSlide = hardwareMap.get(DcMotorEx.class, "Left Lift");;
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setTargetPosition(0);
        leftSlide.setPower(1);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide = hardwareMap.get(DcMotorEx.class, "Right Lift");
//        rightSlide.setDirection(DcMotor.Direction.REVERSE); //V2
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setTargetPosition(0);
        rightSlide.setPower(1);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slideSetPos(int pos) {
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
    }

    public double getPow() {
        if (leftSlide.getPower() == rightSlide.getPower()) {
            return leftSlide.getPower();
        } else {
            return (leftSlide.getPower() + rightSlide.getPower()) / 2;
        }
    }

    public double getLeftPow() {return leftSlide.getPower();}
    public double getRightPow() {return rightSlide.getPower();}

    public int getRightPos() {
        return rightSlide.getCurrentPosition();
    }

    public int getLeftPos() {
        return leftSlide.getCurrentPosition();
    }

    public void setPow(double pow) {
        leftSlide.setPower(pow);
        rightSlide.setPower(pow);
    }
}
