package org.firstinspires.ftc.teamcode.roadRunnerActions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.Constants.pivotTickPerDegree;
import static org.firstinspires.ftc.teamcode.Constants.slideTickPerIn;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    private DcMotor liftMotor = null;
    private Servo claw;
    private Servo score;
    private Servo axis;
    private DcMotor pivot = null;
    public Lift(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivot = hardwareMap.get(DcMotor.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");

        score = hardwareMap.get(Servo.class, "score");

        axis = hardwareMap.get(Servo.class, "axis");
    }


    //High Basket Score:
    public class LiftHighBasket implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftMotor.setPower(.5);
                liftMotor.setTargetPosition((int)(31 * slideTickPerIn));
                initialized = true;
            }
            return true;
        }
    }

    public Action liftHighBasket() {
        return new LiftHighBasket();
    }


    //High Specimen Score:
    public class LiftHighSpec implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftMotor.setPower(.5);
                liftMotor.setTargetPosition((int)(20 * slideTickPerIn));
                initialized = true;
            }
            return true;
        }
    }

    public Action liftHighSpec() {
        return new LiftHighSpec();
    }


    //Pivot Up:
    public class PivotUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                pivot.setPower(.7);
                pivot.setTargetPosition(340);
                initialized = true;
            }
            return true;
        }
    }

    public Action pivotUp() {
        return new PivotUp();
    }


    //Pivot Down:
    public class PivotDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                pivot.setPower(.7);
                pivot.setTargetPosition(0);
                initialized = true;
            }
            return true;
        }
    }

    public Action pivotDown() {
        return new PivotDown();
    }
}
