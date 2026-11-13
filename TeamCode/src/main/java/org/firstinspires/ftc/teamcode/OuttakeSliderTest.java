package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OuttakeSliderTest", group = "Linear Opmode")
public class OuttakeSliderTest extends LinearOpMode {

    private Servo outtakeSlider = null;
    private double currentPosition = 0.2; // Start at a middle position (0.0 to 1.0)

    // Flag variables to prevent continuous triggering while holding the button
    private boolean aButtonPrevState = false;
    private boolean yButtonPrevState = false;

    @Override
    public void runOpMode() {
        // Initialize the servo hardware map (name must match the config on the Driver Station)
        outtakeSlider = hardwareMap.get(Servo.class, "outtakeSlider");

        // Set initial position
        outtakeSlider.setPosition(currentPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean aButtonCurrState = gamepad1.a;
            boolean yButtonCurrState = gamepad1.y;

            // Move "forward" (increase position) when 'A' is newly pressed
            if (aButtonCurrState && !aButtonPrevState) {
                currentPosition += 0.1; // Adjust this increment value (0.0 to 1.0 scale)
                // Keep position within safe limits [0.0, 1.0]
                if (currentPosition > 1.0) {
                    currentPosition = 1.0;
                }
                outtakeSlider.setPosition(currentPosition);
            }

            // Move "forward/next step" (or adjust) when 'Y' is newly pressed
            if (yButtonCurrState && !yButtonPrevState) {
                currentPosition -= 0.1; // Adjust increment value
                if (currentPosition > 1.0) {
                    currentPosition = 1.0;
                }
                outtakeSlider.setPosition(currentPosition);
            }

            // Save current button states for the next loop cycle
            aButtonPrevState = aButtonCurrState;
            yButtonPrevState = yButtonCurrState;

            // Feedback on Driver Station screen
            telemetry.addData("Servo Position", currentPosition);
            telemetry.update();
        }
    }
}
