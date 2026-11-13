package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//added comment

//Ben is adding a comment
//colins comment
@TeleOp(name="Field Centric TeleOp Tutorial")
public class FieldCentricDriveTutorial extends LinearOpMode {
    private Servo turnTableServo = null;
    private double turntablePosition = 0.75;
    private TrackTag tagTracker = null;
   private StateMachine stateMachine = null;
    private GoBildaPinpointDriver odo;

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    // TODO: Adjust this value to match your motor encoder counts per inch of travel!
    //final double COUNTS_PER_INCH = 453.7; // Example for a GOBUG/REV/AndyMark motor with standard ticks
    final double COUNTS_PER_INCH = 362.9; // Example for a GOBUG/REV/AndyMark motor with standard ticks

    double TURNTABLE_SPEED = 0.01;
    @Override
    public void runOpMode() {


        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");

        // Set directions
//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
//        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // CHANGE THIS: Reverse the right side motors instead of the left
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize using the absolute package path
        odo = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-114.3, -171.45, DistanceUnit.MM);

        // Set odometry pod resolution for the swingarm (spring) pod
        odo.setEncoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // Set directions
        odo.setEncoderDirections(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD,
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
// make turntable servo
        turnTableServo = hardwareMap.get(Servo.class, "tableTopServo");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //track tag
        tagTracker = new TrackTag(hardwareMap, 0);
        //state machine
        stateMachine = new StateMachine();

        tagTracker.start();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad1.dpad_left) {
                turntablePosition -= TURNTABLE_SPEED;
            } else if (gamepad1.dpad_right) {
                turntablePosition += TURNTABLE_SPEED;
            }

            // Drive Train Logic
            double forward = -gamepad1.left_stick_y;  //Remember, y is reversed
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;  //Rotation

            //update returns postion that turn table servo is set to
            //turntablePosition = tagTracker.update(turntablePosition);
            // Keep the position strictly within the 0.0 to 1.0 boundaries
//            if (turntablePosition < 0.0) turntablePosition = 0.0;
//            if (turntablePosition > 1.0) turntablePosition = 1.0;


//            Pose2D pos = odo.getPosition();
//            double heading = pos.getHeading(AngleUnit.RADIANS);

//            double cosAngle = Math.cos((Math.PI / 2) - heading);
//            double sinAngle = Math.sin((Math.PI / 2) - heading);
//
//            double globalStrafe = -forward * sinAngle + strafe * cosAngle;
//            double globalForward = forward * cosAngle + strafe * sinAngle;
//
//            double frontLeftPower =  globalForward + globalStrafe + rotate;
//            double frontRightPower =  globalForward - globalStrafe - rotate;
//            double backLeftPower =  globalForward - globalStrafe + rotate;
//            double backRightPower =  globalForward + globalStrafe - rotate;
//
//            frontLeftDrive.setPower(frontLeftPower);
//            backLeftDrive.setPower(backLeftPower);
//            frontRightDrive.setPower(frontRightPower);
//            backRightDrive.setPower(backRightPower);

            // --- STEP B STARTS HERE ---
            // 1. Tell the pinpoint computer to grab the latest sensor data
            odo.update();

            // 2. Fetch the updated heading
            Pose2D pos = odo.getPosition();
            double heading = pos.getHeading(AngleUnit.RADIANS);

            // 3. Translate your stick inputs relative to the field's angle
            double globalStrafe = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            double globalForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

            // 4. Balance and distribute the powers to the wheels
            double denominator = Math.max(Math.abs(globalForward) + Math.abs(globalStrafe) + Math.abs(rotate), 1.0);
            double frontLeftPower =  (globalForward + globalStrafe + rotate) / denominator;
            double backLeftPower =   (globalForward - globalStrafe + rotate) / denominator;
            double frontRightPower = (globalForward - globalStrafe - rotate) / denominator;
            double backRightPower =  (globalForward + globalStrafe - rotate) / denominator;

            // 5. Send the fixed powers to your physical motors
            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            turnTableServo.setPosition(turntablePosition);

// --- STATE MACHINE ---
            StateMachine.State state = stateMachine.update(
                    tagTracker.hasTarget(),
                    tagTracker.getTx(),
                    tagTracker.getTy());

            telemetry.addData("State", state);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.addData("Tag Visible", tagTracker.hasTarget());
            telemetry.addData("tx (deg)", tagTracker.getTx());
            telemetry.addData("ty (deg)", tagTracker.getTy());
            telemetry.addData("turn table position", turntablePosition);
            telemetry.update();

        }
    }
}