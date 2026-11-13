package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx; // Required for advanced PWM controls


@TeleOp(name = "IntakeOutakeOpMode", group = "Linear OpMode")
public class IntakeOutakeOpMode extends LinearOpMode {

    // Declare hardware variables
    private CRServo andymarkIntake;   // Continuous Rotation Servo

    private CRServo andymarkServoOuttakeFeeder;

    //third intake
    private CRServo andymarkIntakeThird;



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


    private double secondArmPosition = .55;
    private double SECOND_ARM_SPEED = .005;


    // Slider Hardware
    private Servo outtakeSlider = null;
    private double currentSliderPosition = 0.3;


    private Servo servoSecondArm = null;

    // Motor Hardware
    private DcMotor outtake = null;
    private DcMotor feederMotor = null;

    // Default target speeds for the systems when triggered
    private double motorMaxPower = 1.0;

    // Button state variables to act as debounces
    private boolean aButtonPrevState = false;
    private boolean yButtonPrevState = false;
    private boolean dpadUpPrevState = false;
    private boolean dpadDownPrevState = false;

    public void runOpMode() {
        // Initialize slider hardware
        outtakeSlider = hardwareMap.get(Servo.class, "outtakeSlider");
        outtakeSlider.setPosition(currentSliderPosition);

        // Initialize motor hardware (Names must match Driver Station configuration exactly)
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");

        // Initialize hardware (Names must exactly match your robot configuration screen)
        andymarkIntake = hardwareMap.get(CRServo.class, "intakeServo");
        gobildaArm = hardwareMap.get(ServoImplEx.class, "armServo");

        andymarkIntakeThird = hardwareMap.get(CRServo.class, "intakeservo3");

        servoSecondArm = hardwareMap.get(ServoImplEx.class, "servoSecondArm");

        andymarkServoOuttakeFeeder = hardwareMap.get(CRServo.class, "servoOuttakeFeeder");

        // Set the arm to its initial starting position before the match starts
        //gobildaArm.setPosition(currentArmPosition);

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
                andymarkServoOuttakeFeeder.setPower(-1.0);
                andymarkIntakeThird.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                andymarkIntake.setPower(-1.0);
                andymarkServoOuttakeFeeder.setPower(1.0);
                andymarkIntakeThird.setPower(-1.0);
            } else {
                andymarkIntake.setPower(0.0);
                andymarkServoOuttakeFeeder.setPower(0);
                andymarkIntakeThird.setPower(0);
            }

            // ==========================================
            // 2. GOBILDA POSITION ARM CONTROL (WITH DE-ENERGIZE)
            // ==========================================
            if (gamepad2.b) {
                // Instantly cut power to the servo motor so it drops and goes completely loose
                gobildaArm.setPwmDisable();
            } else {
                // If D-pad is used to manually jog the arm, re-enable power first
                if (gamepad2.dpad_up) {
                    gobildaArm.setPwmEnable();
                    currentArmPosition += ARM_SPEED;
                } else if (gamepad2.dpad_down) {
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
               // gobildaArm.setPosition(currentArmPosition);
            }

            //Second Arm Position
            if (gamepad2.y) {
                servoSecondArm.setPosition(secondArmPosition);
                secondArmPosition += SECOND_ARM_SPEED;
            } else if (gamepad2.a) {
                servoSecondArm.setPosition(secondArmPosition);
                secondArmPosition -= SECOND_ARM_SPEED;
            }


            //OUTTAKE Code
            // Read digital button values from Gamepad 1
            boolean aButtonCurrState = gamepad1.a;
            boolean yButtonCurrState = gamepad1.y;
            boolean xButtonPressed = gamepad1.x;
            boolean bButtonPressed = gamepad1.b;
            boolean dpadUpCurrState = gamepad1.dpad_up;
            boolean dpadDownCurrState = gamepad1.dpad_down;

            /* ==================== OUTTAKE SLIDER POSITIONS ==================== */
            // Button A: Increase position (moves slider forward)
            if (aButtonCurrState && !aButtonPrevState) {
                currentSliderPosition += 0.1;
                if (currentSliderPosition > 1.0) currentSliderPosition = 1.0;
                outtakeSlider.setPosition(currentSliderPosition);
            }

            // Button Y: Decrease position (moves slider backward)
            if (yButtonCurrState && !yButtonPrevState) {
                currentSliderPosition -= 0.1;
                if (currentSliderPosition < 0.0) currentSliderPosition = 0.0;
                outtakeSlider.setPosition(currentSliderPosition);
            }

            /* ==================== ARROW SPEED ADJUSTMENTS ==================== */
            // Up Arrow: Increases global system speed ceiling by increments of 0.1
            if (dpadUpCurrState && !dpadUpPrevState) {
                motorMaxPower += 0.1;
                if (motorMaxPower > 1.0) motorMaxPower = 1.0;
            }

            // Down Arrow: Decreases global system speed ceiling by increments of 0.1
            if (dpadDownCurrState && !dpadDownPrevState) {
                motorMaxPower -= 0.1;
                if (motorMaxPower < 0.0) motorMaxPower = 0.0;
            }

            /* ==================== FEEDER & OUTTAKE SYSTEM ==================== */
            if (xButtonPressed) {
                // If X is pressed, run both motors FORWARD at our speed ceiling
                outtake.setPower(motorMaxPower);
                feederMotor.setPower(-1);
            } else if (bButtonPressed) {
                // If B is pressed, run both motors BACKWARD (Negative power)
                outtake.setPower(-motorMaxPower);
                feederMotor.setPower(1);
            } else {
                // System is idle when neither X nor B are pressed
                outtake.setPower(0);
                feederMotor.setPower(0);
            }

            /* ==================== SAVING BUTTON STATES ==================== */
            aButtonPrevState = aButtonCurrState;
            yButtonPrevState = yButtonCurrState;
            dpadUpPrevState = dpadUpCurrState;
            dpadDownPrevState = dpadDownCurrState;

            /* ==================== DRIVER STATION TELEMETRY ==================== */
            telemetry.addData("Slider Position (A/Y)", currentSliderPosition);
            telemetry.addData("System Set Max Power (Up/Down)", "%.1f", motorMaxPower);

            String status = "STOPPED";
            if (xButtonPressed) status = "RUNNING FORWARD";
            if (bButtonPressed) status = "RUNNING REVERSE";
            telemetry.addData("System Active Running Status", status);



            // ==========================================
            // 3. TELEMETRY
            // ==========================================
            telemetry.addData("Intake Power 1", andymarkIntake.getPower());
            telemetry.addData("Intake Power 2", andymarkServoOuttakeFeeder.getPower());
            telemetry.addData("Intake Power 3", andymarkIntakeThird.getPower());

            telemetry.addData("Intake Feeder Motor Middle", servoSecondArm.getPosition());

            telemetry.addData("Arm Power Status", gobildaArm.isPwmEnabled() ? "ENABLED (Taut)" : "DISABLED (Loose)");
            telemetry.addData("Arm Target Position", currentArmPosition);
            telemetry.addData("Arm Actual Position", gobildaArm.getPosition());
            telemetry.update();
        }
    }
}