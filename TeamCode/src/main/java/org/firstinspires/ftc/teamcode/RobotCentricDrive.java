package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Robot Centric Drive")
public class RobotCentricDrive extends LinearOpMode {

    ElapsedTime loopTimer = new ElapsedTime();

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    // Turntable servo variables
    private Servo turnTableServo;
    private double turntablePosition = .74;      // 0.74 poin// ts the outtake towards the front.
    private final double STEP_SIZE = 0.02;       // 0.02 = ~3

    // TODO: Adjust these two limits to match your turntable's safe physical travel range!
    private final double MIN_TURNTABLE_POS = 0.35;
    private final double MAX_TURNTABLE_POS = 1.00;

    // 6 degrees per click on a 5-turn servo
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;

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

        // Working motor directions from your test
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

        //let's make sure we center the table top to point to the front.
        turnTableServo.setPosition(turntablePosition);

        // Initialize Limelight tracker and state machine
        tagTracker = new TrackTag(hardwareMap, RED_ALLIANCE_PIPELINE);
        stateMachine = new StateMachine();


        telemetry.addData("Status", "Initialized - Servo at " + turntablePosition);
        telemetry.update();

        waitForStart();

        tagTracker.start();

        while (opModeIsActive()) {
            // 1. Automatic AprilTag Tracking Update
            // (Assumes your TrackTag.update() method takes current position and returns the new target position)
            turntablePosition = tagTracker.update(turntablePosition);

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

            // Telemetry to track your exact servo position
            //telemetry.addData("Turntable Pos", "%.3f (%.0f deg)", turntablePosition, turntablePosition * 1800);

            // 6. Telemetry
            telemetry.addData("State", state);

            // --- TELEMETRY ---
            int activePipe = tagTracker.getActivePipeline();
            String allianceLabel = (activePipe == RED_ALLIANCE_PIPELINE) ? "RED (0)"
                    : (activePipe == BLUE_ALLIANCE_PIPELINE) ? "BLUE (1)"
                      : "UNKNOWN (" + activePipe + ")";

            boolean atLimit = (turntablePosition <= MIN_TURNTABLE_POS + 0.001)
                    || (turntablePosition >= MAX_TURNTABLE_POS - 0.001);

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
            loopTimer.reset();
            telemetry.update();

            telemetry.update();
        }
    }
}