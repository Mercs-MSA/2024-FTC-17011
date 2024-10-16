package org.firstinspires.ftc.teamcode.roadRunnerActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climber {
    private DcMotorEx climberR;
    private DcMotorEx climberL;
    private Servo hookR;
    private Servo hookL;
    public Climber(HardwareMap hardwareMap) {
        climberR = hardwareMap.get(DcMotorEx.class, "climberR");
        climberR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberR.setDirection(DcMotor.Direction.FORWARD);
        climberR.setPower(0.5);

        climberL = hardwareMap.get(DcMotorEx.class, "climberL");
        climberL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberL.setDirection(DcMotor.Direction.FORWARD);
        climberL.setPower(0.5);

        hookL = hardwareMap.get(Servo.class, "hookL");
        hookL.setPosition(0.1);

        hookR = hardwareMap.get(Servo.class, "hookR");
        hookR.setPosition(0.1);
    }
    public class ClimberUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                climberR.setTargetPosition(20);
                climberL.setTargetPosition(20);
                hookL.setPosition(0.3);
                hookR.setPosition(0.6);
                initialized = true;
            }
            return true;
        }
    }

    public Action ClimberUp() {
        return new ClimberUp();
    }
}
// within the Lift class
