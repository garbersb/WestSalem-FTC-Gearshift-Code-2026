package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Intake {

    // Continuous Rotation Servos
    private final CRServo andymarkIntake;
    private final CRServo andymarkServoOuttakeFeeder;
   // private final CRServo andymarkIntakeThird;

    // Arm Servos
    //private final ServoImplEx gobildaArm;
    private final Servo servoSecondArm;

    // goBilda arm safety limits & speed
    private static final double ARM_MAX_LIMIT = 0.6;
    private static final double ARM_MIN_LIMIT = 0.5;
    private static final double ARM_SPEED = 0.005;
    private double currentArmPosition = 0.55;

    // Second arm position & spee
    private double secondArmPosition = 0.35;

    private static final double SECOND_ARM_STEP = 0.02; // Exact change per button press
    private boolean y2WasPressed = false;
    private boolean a2WasPressed = false;

    public Intake(HardwareMap hardwareMap) {
        andymarkIntake = hardwareMap.get(CRServo.class, "intakeServo");
        //andymarkIntakeThird = hardwareMap.get(CRServo.class, "intakeservo3");
        andymarkServoOuttakeFeeder = hardwareMap.get(CRServo.class, "servoOuttakeFeeder");

        //gobildaArm = hardwareMap.get(ServoImplEx.class, "armServo");
        servoSecondArm = hardwareMap.get(Servo.class, "servoSecondArm");

        // Uncomment if you want the arm to snap to start position on init:
        // gobildaArm.setPosition(currentArmPosition);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        // 1. Continuous Intake Control (Gamepad 1 Bumpers)
        if (gamepad2.left_bumper) {
            andymarkIntake.setPower(1.0);
            andymarkServoOuttakeFeeder.setPower(-1.0);
            //andymarkIntakeThird.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            andymarkIntake.setPower(-1.0);
            andymarkServoOuttakeFeeder.setPower(1.0);
            //andymarkIntakeThird.setPower(-1.0);
        } else {
            andymarkIntake.setPower(0.0);
            andymarkServoOuttakeFeeder.setPower(0.0);
            //andymarkIntakeThird.setPower(0.0);
        }

        // 2. goBilda Position Arm Control with De-Energize (Gamepad 2)
//        if (gamepad2.b) {
//            gobildaArm.setPwmDisable();
//        } else {
//            if (gamepad2.dpad_up) {
//                gobildaArm.setPwmEnable();
//                currentArmPosition += ARM_SPEED;
//            } else if (gamepad2.dpad_down) {
//                gobildaArm.setPwmEnable();
//                currentArmPosition -= ARM_SPEED;
//            }
//
//            currentArmPosition = Math.max(ARM_MIN_LIMIT, Math.min(ARM_MAX_LIMIT, currentArmPosition));
//            // Uncomment when ready to actively command position:
//            // gobildaArm.setPosition(currentArmPosition);
//        }

        // 3. Second Arm Position Control (Gamepad 2 Y/A)
        if (gamepad2.y && !y2WasPressed) {
            secondArmPosition = Math.min(1.0, secondArmPosition + SECOND_ARM_STEP);
            servoSecondArm.setPosition(secondArmPosition);
        } else if (gamepad2.a && !a2WasPressed) {
            secondArmPosition = Math.max(0.0, secondArmPosition - SECOND_ARM_STEP);
            servoSecondArm.setPosition(secondArmPosition);
        }

        y2WasPressed = gamepad2.y;
        a2WasPressed = gamepad2.a;
    }

    public double getArmPosition() {
        return currentArmPosition;
    }

    public double getSecondArmPosition() {
        return secondArmPosition;
    }

    public void runIntake(double power) {
        andymarkIntake.setPower(power);
        andymarkServoOuttakeFeeder.setPower(-power);
        //andymarkIntakeThird.setPower(power);
    }

    public void stop() {
        runIntake(0.0);
    }
}
