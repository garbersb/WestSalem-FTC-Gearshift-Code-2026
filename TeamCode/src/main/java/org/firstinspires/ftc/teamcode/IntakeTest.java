package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImplEx; // Required for advanced PWM controls

@TeleOp(name = "IntakeAndArmControl", group = "Linear OpMode")
public class IntakeTest extends LinearOpMode {

    // Declare hardware variables
    private CRServo andymarkIntake;   // Continuous Rotation Servo

    private CRServo andymarkServoOuttakeFeeder;


    private ServoImplEx gobildaArm;   // Extended Positional Servo for de-energizing

    // Define goBilda arm safety limits (0.0 to 1.0)
    private static final double ARM_MAX_LIMIT = 0.6;
    private static final double ARM_MIN_LIMIT = 0.5;

    // Variable to track the current target position of the arm
    // Initialized safely between 0.5 and 0.6 to prevent instant snapping on start
    private double currentArmPosition = 0.55;

    // Adjust this value to change the movement speed!
    // Higher number = faster movement. Smaller number = slower movement.
    private static final double ARM_SPEED = 0.005;



    @Override
    public void runOpMode() {

        // Initialize hardware (Names must exactly match your robot configuration screen)
        andymarkIntake = hardwareMap.get(CRServo.class, "intakeServo");
        gobildaArm = hardwareMap.get(ServoImplEx.class, "armServo");


        andymarkServoOuttakeFeeder = hardwareMap.get(CRServo.class, "servoOuttakeFeeder");

        // Set the arm to its initial starting position before the match starts
        gobildaArm.setPosition(currentArmPosition);

        // Send simple text telemetry
        telemetry.addData("Status", "Hardware Mapped Cleanly.");
        telemetry.addData("Ready", "Press INIT to test start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ==========================================
            // 1. ANDYMARK CONTINUOUS INTAKE CONTROL
            // ==========================================
            // Left bumper intakes, Right bumper outputs, otherwise stop
            if (gamepad1.left_bumper) {
                andymarkIntake.setPower(1.0);
                andymarkServoOuttakeFeeder.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                andymarkIntake.setPower(-1.0);
                andymarkServoOuttakeFeeder.setPower(-1.0);
            } else {
                andymarkIntake.setPower(0.0);
                andymarkServoOuttakeFeeder.setPower(0);
            }

            // ==========================================
            // 2. GOBILDA POSITION ARM CONTROL (WITH DE-ENERGIZE)
            // ==========================================
            if (gamepad1.b) {
                // Instantly cut power to the servo motor so it drops and goes completely loose
                gobildaArm.setPwmDisable();
            } else {
                // If D-pad is used to manually jog the arm, re-enable power first
                if (gamepad1.dpad_up) {
                    gobildaArm.setPwmEnable();
                    currentArmPosition += ARM_SPEED;
                } else if (gamepad1.dpad_down) {
                    gobildaArm.setPwmEnable();
                    currentArmPosition -= ARM_SPEED;
                }

                // Enforce safety thresholds
                if (currentArmPosition > ARM_MAX_LIMIT) {
                    currentArmPosition = ARM_MAX_LIMIT;
                }
                if (currentArmPosition < ARM_MIN_LIMIT) {
                    currentArmPosition = ARM_MIN_LIMIT;
                }

                // Apply the safe, updated position to the servo
                gobildaArm.setPosition(currentArmPosition);
            }

            // ==========================================
            // 3. TELEMETRY
            // ==========================================
            telemetry.addData("Intake Power", andymarkIntake.getPower());
            telemetry.addData("Arm Power Status", gobildaArm.isPwmEnabled() ? "ENABLED (Taut)" : "DISABLED (Loose)");
            telemetry.addData("Arm Target Position", currentArmPosition);
            telemetry.addData("Arm Actual Position", gobildaArm.getPosition());
            telemetry.update();
        }
    }
}