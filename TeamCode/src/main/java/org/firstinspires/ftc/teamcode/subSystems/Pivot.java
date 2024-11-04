package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Pivot {
    private DcMotorEx pivot;

    public Pivot(HardwareMap hardwareMap, double p, double i, double d, double f) throws InterruptedException {
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setTargetPositionTolerance(3);
        PIDFCoefficients pivotPIDFNew = new PIDFCoefficients(p, i, d, f);
        pivot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pivotPIDFNew);
        pivot.setTargetPosition(0);
        pivot.setPower(.85);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void pivotSetPos(int pos) {
        pivot.setTargetPosition(pos);
    }

    public void setPow(double pow) {
        pivot.setPower(pow);
    }
    public int getPos() {
        return pivot.getCurrentPosition();
    }
}
