package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.teamcode.Constants.pivotUpPos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Pivot {
//    private DcMotorEx pivot; //V2
    private DcMotorEx rightPivot; //V3
    private DcMotorEx leftPivot; //V3

    public Pivot(HardwareMap hardwareMap, double p, double i, double d, double f) throws InterruptedException {
//        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
//        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        pivot.setTargetPositionTolerance(3);
//        PIDFCoefficients pivotPIDFNew = new PIDFCoefficients(p, i, d, f);
//        pivot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pivotPIDFNew);
//        pivot.setTargetPosition(0);
//        pivot.setPower(.85);
//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightPivot = hardwareMap.get(DcMotorEx.class, "rightPivot");
        rightPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPivot.setTargetPositionTolerance(3);
        PIDFCoefficients pivotPIDFNew = new PIDFCoefficients(p, i, d, f);
        rightPivot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pivotPIDFNew);
        rightPivot.setTargetPosition(0);
        rightPivot.setPower(.5);
        rightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftPivot = hardwareMap.get(DcMotorEx.class, "leftPivot");
        leftPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPivot.setTargetPositionTolerance(3);
        leftPivot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pivotPIDFNew);
        leftPivot.setTargetPosition(0);
        leftPivot.setPower(.5);
        leftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void pivotSetPos(int pos) {
        rightPivot.setTargetPosition(pos);
        leftPivot.setTargetPosition(pos);
    }
    public void setPow(double pow) {
        rightPivot.setPower(pow);
        leftPivot.setPower(pow);
    }
    public int getPos() {
        return leftPivot.getCurrentPosition();
    }
    public double getLeftPow() {return leftPivot.getPower();}
    public double getRightPow() {return rightPivot.getPower();}


    public void resetPos() {
        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void forceDown() {
        rightPivot.setTargetPosition(-1000);
        leftPivot.setTargetPosition(-1000);
    }

    public void normalMode() {
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void encoderMode(double p, double i, double d, double f) {
        rightPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPivot.setTargetPositionTolerance(3);
        PIDFCoefficients pivotPIDFNew = new PIDFCoefficients(p, i, d, f);
        rightPivot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pivotPIDFNew);
        rightPivot.setTargetPosition(0);
        rightPivot.setPower(.65);
        rightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPivot.setTargetPositionTolerance(3);
        leftPivot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pivotPIDFNew);
        leftPivot.setTargetPosition(0);
        leftPivot.setPower(.65);
        leftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
