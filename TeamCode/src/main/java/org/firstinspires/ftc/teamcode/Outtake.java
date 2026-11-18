package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    private final CRServo andymarkIntakeThird;

    // Hardware
    private final Servo outtakeSlider;
    private final DcMotorEx outtake;
    private final DcMotor feederMotor;

    // Max velocity in ticks/sec (NeveRest 1:1 Hex Motor max is ~2800 ticks/sec)
    private static final double MAX_VELOCITY_TICKS = 2800.0;

    // State variables
    private double currentSliderPosition = 0.3;
    private double motorMaxPower = 1.0;

    // Debounce flags
    private boolean aButtonPrevState = false;
    private boolean yButtonPrevState = false;
    private boolean dpadUpPrevState = false;
    private boolean dpadDownPrevState = false;

    public Outtake(HardwareMap hardwareMap) {
        outtakeSlider = hardwareMap.get(Servo.class, "outtakeSlider");
        outtakeSlider.setPosition(currentSliderPosition);

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");

        andymarkIntakeThird = hardwareMap.get(CRServo.class, "intakeservo3");
    }

    public void update(Gamepad gamepad1) {
        boolean aButtonCurrState = gamepad1.a;
        boolean yButtonCurrState = gamepad1.y;
        boolean xButtonPressed = gamepad1.x;
        boolean bButtonPressed = gamepad1.b;
        boolean dpadUpCurrState = gamepad1.dpad_up;
        boolean dpadDownCurrState = gamepad1.dpad_down;

        // 1. Outtake Slider Positions (A = forward, Y = backward)
        if (aButtonCurrState && !aButtonPrevState) {
            currentSliderPosition = Math.min(1.0, currentSliderPosition + 0.1);
            outtakeSlider.setPosition(currentSliderPosition);
        }
        if (yButtonCurrState && !yButtonPrevState) {
            currentSliderPosition = Math.max(0.0, currentSliderPosition - 0.1);
            outtakeSlider.setPosition(currentSliderPosition);
        }

        // 2. Speed Ceiling Adjustments (D-Pad Up/Down)
        if (dpadUpCurrState && !dpadUpPrevState) {
            motorMaxPower = Math.min(1.0, motorMaxPower + 0.1);
        }
        if (dpadDownCurrState && !dpadDownPrevState) {
            motorMaxPower = Math.max(0.0, motorMaxPower - 0.1);
        }

        // 3. Feeder & Outtake Motors (X = shoot forward, B = reverse, else stop)
        double targetVelocity = motorMaxPower * MAX_VELOCITY_TICKS;

        if (xButtonPressed) {
            outtake.setVelocity(targetVelocity);
            feederMotor.setPower(-1.0);
            andymarkIntakeThird.setPower(1.0);
        } else if (bButtonPressed) {
            outtake.setVelocity(-targetVelocity);
            feederMotor.setPower(1.0);
            andymarkIntakeThird.setPower(-1.0);
        } else {
            outtake.setVelocity(0.0);
            feederMotor.setPower(0.0);
            andymarkIntakeThird.setPower(0.0);
        }

        // Save button states for next loop
        aButtonPrevState = aButtonCurrState;
        yButtonPrevState = yButtonCurrState;
        dpadUpPrevState = dpadUpCurrState;
        dpadDownPrevState = dpadDownCurrState;
    }

    // --- AUTONOMOUS HELPER METHODS ---

    public void setTargetVelocity(double ticksPerSec) {
        double clampedVel = Math.max(-MAX_VELOCITY_TICKS, Math.min(MAX_VELOCITY_TICKS, ticksPerSec));
        outtake.setVelocity(clampedVel);
    }

    public void feedToShooter(double power) {
        feederMotor.setPower(-power);
    }

    public void setSliderPosition(double position) {
        currentSliderPosition = Math.max(0.0, Math.min(1.0, position));
        outtakeSlider.setPosition(currentSliderPosition);
    }

    public void stop() {
        outtake.setVelocity(0.0);
        feederMotor.setPower(0.0);
    }

    // --- GETTERS ---

    public double getMotorMaxPower() {
        return motorMaxPower;
    }

    public double getOuttakeVelocity() {
        return outtake.getVelocity();
    }

    public double getSliderPosition() {
        return currentSliderPosition;
    }
}