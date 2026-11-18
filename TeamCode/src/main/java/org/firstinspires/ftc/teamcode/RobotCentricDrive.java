package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Robot Centric Drive")
public class RobotCentricDrive extends LinearOpMode {

    // Shared tracking toggle: can be toggled on Gamepad 1 (X button) and read by redAuto / blueAuto
    public static boolean autoTrackingEnabled = true;

    private Intake intake = null;
    private Outtake outtake = null;

    ElapsedTime loopTimer = new ElapsedTime();

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    // Turntable servo variables
    private Servo turnTableServo;
    private double turntablePosition = .74;      // 0.74 points the outtake towards the front
    private final double STEP_SIZE = 0.02;       // 0.02 = ~36 deg on a 5-turn servo

    // Safe physical travel range for turntable
    private final double MIN_TURNTABLE_POS = 0.35;
    private final double MAX_TURNTABLE_POS = 1.00;

    // Button debounce flags
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;
    private boolean x1WasPressed = false;

    // Limelight tracker & State Machine
    private TrackTag tagTracker = null;
    private StateMachine stateMachine = null;

    private static int RED_ALLIANCE_PIPELINE = 0;
    private static int BLUE_ALLIANCE_PIPELINE = 1;

    @Override
    public void runOpMode() {
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");

        // Working motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize turntable servo and move to starting center position
        turnTableServo = hardwareMap.get(Servo.class, "tableTopServo");
        turnTableServo.setPosition(turntablePosition);

        // Initialize Limelight tracker and state machine
        tagTracker = new TrackTag(hardwareMap, RED_ALLIANCE_PIPELINE);
        stateMachine = new StateMachine();

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        // Allow toggling tracking with Gamepad 1 'X' while waiting for Start
        while (!isStarted() && !isStopRequested()) {
            handleTrackingToggle();
            telemetry.addData("Status", "Initialized - Servo at " + turntablePosition);
            telemetry.addData("Auto/TeleOp Tracking [GP1 X]", autoTrackingEnabled ? "ON" : "OFF");
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        tagTracker.start();

        while (opModeIsActive()) {
            // Check Gamepad 1 'X' to toggle AprilTag tracking ON/OFF
            handleTrackingToggle();

            intake.update(gamepad1, gamepad2);
            outtake.update(gamepad2);

            // 1. Automatic AprilTag Tracking Update (only moves turntable if tracking is ON)
            if (autoTrackingEnabled) {
                turntablePosition = tagTracker.update(turntablePosition);
            }

            // 2. Manual D-Pad Override (Optional fine-tuning or re-centering)
            if (gamepad1.dpad_up && !dpadUpWasPressed) {
                turntablePosition += STEP_SIZE;
            }
            if (gamepad1.dpad_down && !dpadDownWasPressed) {
                turntablePosition -= STEP_SIZE;
            }
            if (gamepad1.a) {
                turntablePosition = 0.74; // Press 'A' to snap back to straight ahead
            }
            dpadUpWasPressed = gamepad1.dpad_up;
            dpadDownWasPressed = gamepad1.dpad_down;

            // 3. Enforce Safe Physical Limits so wires never tangle or hit stops
            turntablePosition = Math.max(MIN_TURNTABLE_POS, Math.min(MAX_TURNTABLE_POS, turntablePosition));
            turnTableServo.setPosition(turntablePosition);

            // 4. Update State Machine
            StateMachine.State state = stateMachine.update(
                    tagTracker.hasTarget(),
                    tagTracker.getTx(),
                    tagTracker.getTy());

            // --- DRIVE TRAIN LOGIC ---
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double rotate  = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);
            double frontLeftPower  = (forward + strafe + rotate) / denominator;
            double backLeftPower   = (forward - strafe + rotate) / denominator;
            double frontRightPower = (forward - strafe - rotate) / denominator;
            double backRightPower  = (forward + strafe - rotate) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            // --- TELEMETRY ---
            int activePipe = tagTracker.getActivePipeline();
            String allianceLabel = (activePipe == RED_ALLIANCE_PIPELINE) ? "RED (0)"
                    : (activePipe == BLUE_ALLIANCE_PIPELINE) ? "BLUE (1)"
                      : "UNKNOWN (" + activePipe + ")";

            boolean atLimit = (turntablePosition <= MIN_TURNTABLE_POS + 0.001)
                    || (turntablePosition >= MAX_TURNTABLE_POS - 0.001);

            telemetry.addData("Tracking [GP1 X]", autoTrackingEnabled ? "ON" : "OFF");
            telemetry.addData("Alliance Pipeline", allianceLabel);
            telemetry.addData("State", state);
            telemetry.addData("Tag Visible", tagTracker.hasTarget() ? "YES (ID: " + tagTracker.getTagId() + ")" : "NO");
            telemetry.addData("Tag TX (deg)", "%.2f", tagTracker.getTx());
            telemetry.addData("Tag TY", "%.2f", tagTracker.getTy());
            telemetry.addData("Turntable Pos", "%.3f (%.0f deg)%s",
                    turntablePosition,
                    turntablePosition * 1800,
                    atLimit ? " [AT LIMIT]" : "");

            telemetry.addData("Distance From Hive", tagTracker.getDistanceInches());
            telemetry.addData("Loop Hz", "%.0f", 1.0 / loopTimer.seconds());

            telemetry.addData("Outtake Max Power", "%.1f", outtake.getMotorMaxPower());
            telemetry.addData("Second Arm Pos", "%.2f", intake.getSecondArmPosition());
            telemetry.addData("Outtake Vel (ticks/s)", "%.0f", outtake.getOuttakeVelocity());

            loopTimer.reset();
            telemetry.update();
        }
    }

    /**
     * Helper method that toggles tracking ON/OFF when Gamepad 1 'X' is pressed.
     * Can also be called by redAuto and blueAuto during their Init loops!
     */
    public static void checkGamepadToggle(boolean xPressed, boolean[] wasPressedHolder) {
        if (xPressed && !wasPressedHolder[0]) {
            autoTrackingEnabled = !autoTrackingEnabled;
        }
        wasPressedHolder[0] = xPressed;
    }


    private void handleTrackingToggle() {
        if (gamepad1.x && !x1WasPressed) {
            autoTrackingEnabled = !autoTrackingEnabled;
        }
        x1WasPressed = gamepad1.x;
    }
}