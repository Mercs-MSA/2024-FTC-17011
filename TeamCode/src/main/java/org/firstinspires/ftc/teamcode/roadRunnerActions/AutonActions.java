package org.firstinspires.ftc.teamcode.roadRunnerActions;

import static org.firstinspires.ftc.teamcode.Constants.pivotTickPerDegree;
import static org.firstinspires.ftc.teamcode.Constants.slideTickPerIn;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonActions {
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private Servo claw;
    private Servo clawPivot;
    private Servo IntakeSpin;
    private Servo specimenIntake;
    private DcMotor pivot = null;
    public AutonActions(HardwareMap hardwareMap) {
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivot = hardwareMap.get(DcMotor.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");

        clawPivot = hardwareMap.get(Servo.class, "score");

        IntakeSpin = hardwareMap.get(Servo.class, "axis");
    }


    //High Basket Score:
    public class LiftHighBasket implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                rightSlide.setPower(.5);
                rightSlide.setTargetPosition((int)(31 * slideTickPerIn));

                leftSlide.setPower(.5);
                leftSlide.setTargetPosition((int)(31 * slideTickPerIn));

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
                leftSlide.setPower(.5);
                leftSlide.setTargetPosition((int)(20 * slideTickPerIn));

                rightSlide.setPower(.5);
                rightSlide.setTargetPosition((int)(20 * slideTickPerIn));

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
                pivot.setTargetPosition(0);
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
                pivot.setTargetPosition((int)(30 * pivotTickPerDegree));
                initialized = true;
            }
            return true;
        }
    }

    public Action pivotDown() {
        return new PivotDown();
    }
    public class ClawClose implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                claw.setPosition(0);
                intialized = true;
            }
            return true;
        }
    }

    public Action ClawClose() {
        return new ClawClose();
    }
    public class ClawOpen implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                claw.setPosition(1);
                intialized = true;
            }
            return true;
        }
    }
    public Action ClawOpen() {
        return new ClawOpen();
    }
    public class ClawPivot180 implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                clawPivot.setPosition(1);
                intialized = true;
            }
            return true;
        }
    }
    public Action ClawPivot180() {
        return new ClawPivot180();
    }
    public class ClawPivot0 implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                clawPivot.setPosition(0);
            }
            return true;
        }
    }
    public Action ClawPivot0() {
        return new ClawPivot0();
    }
    public class ClawSpin90 implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                IntakeSpin.setPosition(1);
            }
            return true;
        }
    }
    public Action ClawSpin90() {
        return new ClawSpin90();
    }
    public class ClawSpin0 implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                IntakeSpin.setPosition(0);
            }
            return true;
        }
    }
    public Action ClawSpin0() {
        return new ClawSpin0();
    }
    public class SpecimenClose implements Action {
        private boolean initalized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initalized) {
                specimenIntake.setPosition(1);
            }
            return true;
        }
    }
    public Action SpecimenClose() {
        return new SpecimenClose();
    }
    public class SpecimenOpen implements Action {
        private boolean initalized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initalized) {
                specimenIntake.setPosition(0);
            }
            return true;
        }
    }
    public Action SpecimenOpen() {
        return new SpecimenOpen();
    }
}
