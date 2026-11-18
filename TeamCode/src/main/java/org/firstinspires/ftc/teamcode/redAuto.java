package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "redAuto", group = "BioBuzz")
public class redAuto extends LinearOpMode {

    // Subsystems
    private Intake intake = null;
    private Outtake outtake = null;
    private TrackTag tagTracker = null;
    private StateMachine stateMachine = null;

    // Drivetrain Motors
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    // Turntable Servo
    private Servo turnTableServo;
    private double turntablePosition = 0.74;
    private static final double MIN_TURNTABLE_POS = 0.35;
    private static final double MAX_TURNTABLE_POS = 1.00;

    // Red Alliance Pipeline
    private static final int RED_ALLIANCE_PIPELINE = 0;

    // --- ENCODER CONSTANTS FOR 4-INCH DRIVE ---
    // Standard goBILDA 312 RPM motor (537.7 ticks/rev) with 96mm (3.78 in) Mecanum wheels:
    // 537.7 / (3.78 * PI) = ~45.3 ticks per inch.
    // Adjust TICKS_PER_INCH if you use different motors or wheel sizes.
    private static final double TICKS_PER_INCH = 45.3;
    private static final double DRIVE_DISTANCE_INCHES = 4.0;
    private static final double DRIVE_POWER = 0.35;

    // --- SHOOTER & INTAKE TIMING ---
    private static final double FLYWHEEL_SPINUP_SECONDS = 1.0;
    private static final double INTAKE_SHOOT_SECONDS = 15.0;

    @Override
    public void runOpMode() {
        // 1. Initialize Drivetrain Motors
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRight");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeft");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();

        // 2. Initialize Turntable, Limelight (Red Pipeline = 0), & StateMachine
        turnTableServo = hardwareMap.get(Servo.class, "tableTopServo");
        turnTableServo.setPosition(turntablePosition);

        tagTracker = new TrackTag(hardwareMap, RED_ALLIANCE_PIPELINE);
        stateMachine = new StateMachine();

//        // 3. Initialize Intake & Outtake
//        intake = new Intake(hardwareMap);
//        outtake = new Outtake(hardwareMap);
//
//        telemetry.addData("Status", "redAuto Initialized (Pipeline 0)");
//        telemetry.update();
//
//        waitForStart();
//
//        if (opModeIsActive()) {

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        boolean[] xToggleState = {false};
        while (!isStarted() && !isStopRequested()) {
            RobotCentricDrive.checkGamepadToggle(gamepad1.x, xToggleState);
            telemetry.addData("Status", "blueAuto Initialized (Pipeline 1)");
            telemetry.addData("Tracking [Press GP1 X]", RobotCentricDrive.autoTrackingEnabled ? "ON" : "OFF");
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        if (opModeIsActive()) {
            tagTracker.start();
            ElapsedTime timer = new ElapsedTime();

            // =================================================================
            // STEP 1: Drive forward 4 inches using encoders
            // =================================================================
            driveForwardInches(DRIVE_DISTANCE_INCHES, DRIVE_POWER);

            // =================================================================
            // STEP 2: Spin up main flywheel briefly so first ball shoots cleanly
            // =================================================================
            timer.reset();
            while (opModeIsActive() && timer.seconds() < FLYWHEEL_SPINUP_SECONDS) {
                updateVisionAndTurret();
                double targetVel = tagTracker.getShooterVelocity(tagTracker.getDistanceInches());
                outtake.setTargetVelocity(targetVel);

                telemetry.addData("Step", "Spinning up flywheel");
                telemetry.addData("Flywheel Vel", "%.0f / %.0f", outtake.getOuttakeVelocity(), targetVel);
                telemetry.update();
            }

            // =================================================================
            // STEP 3: Spin all 3 intake servos and both outtake motors
            // =================================================================
            timer.reset();
            while (opModeIsActive() && timer.seconds() < INTAKE_SHOOT_SECONDS) {
                StateMachine.State state = updateVisionAndTurret();

                // Calculate speed from Limelight distance (defaults to 1900 ticks/s if no tag)
                double distance = tagTracker.getDistanceInches();
                double targetVel = tagTracker.getShooterVelocity(distance);

                // Run both outtake motors (flywheel + feederMotor)
                outtake.setTargetVelocity(targetVel);
                outtake.feedToShooter(1.0);

                // Run all 3 intake servos (intakeServo, intakeservo3, servoOuttakeFeeder)
                intake.runIntake(1.0);

                telemetry.addData("Step", "Intaking & Shooting (Red)");
                telemetry.addData("State", state);
                telemetry.addData("Tag ID", tagTracker.getTagId());
                telemetry.addData("Distance (in)", "%.1f", distance);
                telemetry.addData("Flywheel Vel", "%.0f / %.0f", outtake.getOuttakeVelocity(), targetVel);
                telemetry.update();
            }

            // =================================================================
            // STEP 4: Stop all motors and servos cleanly
            // =================================================================
            intake.stop();
            outtake.stop();
            stopDrive();
            tagTracker.stop();
        }
    }

    private void driveForwardInches(double inches, double power) {
        int targetTicks = (int) Math.round(inches * TICKS_PER_INCH);

        resetDriveEncoders();

        frontLeftDrive.setTargetPosition(targetTicks);
        backLeftDrive.setTargetPosition(targetTicks);
        frontRightDrive.setTargetPosition(targetTicks);
        backRightDrive.setTargetPosition(targetTicks);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(Math.abs(power));
        backLeftDrive.setPower(Math.abs(power));
        frontRightDrive.setPower(Math.abs(power));
        backRightDrive.setPower(Math.abs(power));

        ElapsedTime driveSafetyTimer = new ElapsedTime();
        while (opModeIsActive()
                && frontLeftDrive.isBusy()
                && backLeftDrive.isBusy()
                && frontRightDrive.isBusy()
                && backRightDrive.isBusy()
                && driveSafetyTimer.seconds() < 3.0) {

            updateVisionAndTurret();
            telemetry.addData("Step", "Driving forward 4 in");
            telemetry.addData("Ticks", "%d / %d", frontLeftDrive.getCurrentPosition(), targetTicks);
            telemetry.update();
        }

        stopDrive();

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetDriveEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private StateMachine.State updateVisionAndTurret() {
        if (RobotCentricDrive.autoTrackingEnabled) {
            turntablePosition = tagTracker.update(turntablePosition);
            turntablePosition = Math.max(MIN_TURNTABLE_POS, Math.min(MAX_TURNTABLE_POS, turntablePosition));
        } else {
            turntablePosition = 0.74; // Keep straight ahead when tracking is OFF
        }
        turnTableServo.setPosition(turntablePosition);

        return stateMachine.update(
                tagTracker.hasTarget(),
                tagTracker.getTx(),
                tagTracker.getTy()
        );
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
    }
}
