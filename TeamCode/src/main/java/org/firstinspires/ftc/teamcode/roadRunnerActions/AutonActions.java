package org.firstinspires.ftc.teamcode.roadRunnerActions;


import static org.firstinspires.ftc.teamcode.Constants.highBasketPos;
import static org.firstinspires.ftc.teamcode.Constants.highSpecimenPos;
import static org.firstinspires.ftc.teamcode.Constants.intakeHoldPos;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotGrabPos;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinDefault;
import static org.firstinspires.ftc.teamcode.Constants.pivotDownPos;
import static org.firstinspires.ftc.teamcode.Constants.pivotTickPerDegree;
import static org.firstinspires.ftc.teamcode.Constants.slideTickPerIn;
import static org.firstinspires.ftc.teamcode.Constants.specimenHoldPos;
import static org.firstinspires.ftc.teamcode.Constants.specimenScorePos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonActions {
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private Servo intake;
    private Servo intakePivot;
    private Servo intakeSpin;
    private Servo specimenIntake;
    private DcMotor pivot = null;
    public AutonActions(HardwareMap hardwareMap) {
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivot = hardwareMap.get(DcMotor.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(Servo.class, "intake");

        intakePivot = hardwareMap.get(Servo.class, "score");

        intakeSpin = hardwareMap.get(Servo.class, "axis");
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
    public class intakeClose implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                intake.setPosition(0);
                intialized = true;
            }
            return true;
        }
    }

    public Action intakeClose() {
        return new intakeClose();
    }
    public class intakeOpen implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                intake.setPosition(1);
                intialized = true;
            }
            return true;
        }
    }
    public Action intakeOpen() {
        return new intakeOpen();
    }
    public class intakePivot180 implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                intakePivot.setPosition(1);
                intialized = true;
            }
            return true;
        }
    }
    public Action intakePivot180() {
        return new intakePivot180();
    }
    public class intakePivot0 implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                intakePivot.setPosition(0);
            }
            return true;
        }
    }
    public Action intakePivot0() {
        return new intakePivot0();
    }
    public class intakeSpin90 implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                intakeSpin.setPosition(1);
            }
            return true;
        }
    }
    public Action intakeSpin90() {
        return new intakeSpin90();
    }
    public class intakeSpin0 implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                intakeSpin.setPosition(0);
            }
            return true;
        }
    }
    public Action intakeSpin0() {
        return new intakeSpin0();
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

        public Action specimenOpen() {
            return new SpecimenOpen();
        }
    }
    public class ScoreHighBasket implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                rightSlide.setTargetPosition((int) highBasketPos);
                leftSlide.setTargetPosition((int) highBasketPos);
                intakePivot.setPosition(intakePivotScorePos);
                intakeSpin.setPosition(intakeSpinDefault);
                if (rightSlide.getCurrentPosition() == highBasketPos) {
                    intake.setPosition(intakeScorePos);
                } else {
                    intake.setPosition(intakeHoldPos);
                }
            }
            return true;
        }
        public Action scoreHighBasket() {
            return new ScoreHighBasket();
        }
    }
    public class ScoreHighSpecimen implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                rightSlide.setTargetPosition((int)highSpecimenPos);
                leftSlide.setTargetPosition((int)highSpecimenPos);
                intakeSpin.setPosition(intakeSpinDefault);
                if (rightSlide.getCurrentPosition() == highSpecimenPos) {
                    specimenIntake.setPosition(specimenScorePos);
                }
                else {
                    specimenIntake.setPosition(specimenHoldPos);
                }
            }
            return true;
        }
        public Action scoreHighSpecimen() {
            return new ScoreHighSpecimen();
        }

    }
    public class PickUpSample implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                intakePivot.setPosition(intakePivotGrabPos);
                intake.setPosition(intakeScorePos);
                intakeSpin.setPosition(intakeSpinDefault);
                pivot.setTargetPosition((int)pivotDownPos);
                rightSlide.setTargetPosition(0);
                leftSlide.setTargetPosition(0);
            }
            return true;
        }
        public Action pickUpSample() {
            return new PickUpSample();
        }
    }
    public class Grab implements Action {
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!intialized) {
                intake.setPosition(intakeHoldPos);
            }
            return true;
        }
        public Action grab() {
            return new Grab();
        }
    }

    public Action SpecimenOpen() {
        return new SpecimenOpen();
    }
}
