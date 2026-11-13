package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RobotControlOpMode", group = "Linear Opmode")
public class OuttakeSliderOpMode extends LinearOpMode {

    // Slider Hardware
    private Servo outtakeSlider = null;
    private double currentSliderPosition = 0.3;

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

    @Override
    public void runOpMode() {
        // Initialize slider hardware
        outtakeSlider = hardwareMap.get(Servo.class, "outtakeSlider");
        outtakeSlider.setPosition(currentSliderPosition);

        // Initialize motor hardware (Names must match Driver Station configuration exactly)
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");

        telemetry.addData("Status", "Initialized Successfully");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
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
                feederMotor.setPower(-motorMaxPower);
            } else if (bButtonPressed) {
                // If B is pressed, run both motors BACKWARD (Negative power)
                outtake.setPower(-motorMaxPower);
                feederMotor.setPower(motorMaxPower);
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

            telemetry.update();
        }
    }
}