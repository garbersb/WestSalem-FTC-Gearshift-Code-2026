package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import java.util.List;

public class TrackTag {

    // Measure these 3 values on your robot and the BIOBUZZ HIVE!
    private static final double CAMERA_HEIGHT_INCHES = 15.5;   // Floor to center of Limelight lens
    private static final double TAG_HEIGHT_INCHES    = 49.0;   // Floor to center of HIVE AprilTag
    private static final double CAMERA_ANGLE_DEG     = 36.0;   // Upward tilt angle of Limelight


    private final Limelight3A limelight;
    private final ElapsedTime lostTimer = new ElapsedTime();

    private double lastTimestamp = -1.0;

    private static final double KP = 0.0001;           // Even gentler step
    private static final double START_TRACK_DEG = 5.0;  // Only move again if tag drifts > 5 deg off center
    private static final double STOP_TRACK_DEG  = 2.5;  // Lock in and stop moving once within 2.5 deg
    private static final double CENTER_POS = 0.74;     // Turntable center
    private static final double LOST_DELAY_SEC = 0.75; // Wait 0.75s before returning to center
    private static final double RETURN_STEP = 0.0004;  // ~0.7 deg per loop (~140 deg/sec smooth return)

    private int lockedTagId = -1;

    private boolean isLocked = false;

    public TrackTag(HardwareMap hardwareMap, int alliance) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(alliance); // pipeline number set to AprilTag in the Limelight web UI
        limelight.setPollRateHz(100);
    }

    public void start() {
        limelight.start();
        lostTimer.reset();
    }

    public void stop() { limelight.stop(); }

    /** True if the Limelight currently sees a valid target. */
    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    /** Horizontal offset from the crosshair in degrees (0 if no target). */
    public double getTx() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0.0;
    }

    /** Vertical offset from the crosshair in degrees (0 if no target). */
    public double getTy() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }
    public int getActivePipeline() {
        LLResult result = limelight.getLatestResult();
        return (result != null) ? result.getPipelineIndex() : -1;
    }

    /** Returns the primary AprilTag ID seen by the Limelight, or -1 if none. */
    public int getTagId() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> tags = result.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {
                return tags.get(0).getFiducialId();
            }
        }
        return -1;
    }

    /** Returns horizontal distance to the HIVE tag in inches, or -1 if no target is visible. */
    public double getDistanceInches() {
        FiducialResult tag = getLockedTag();
        if (tag != null) {
            double ty = tag.getTargetYDegrees();
            double totalAngleRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);
            if (Math.tan(totalAngleRad) > 0.01) {
                return (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(totalAngleRad);
            }
        }
        return -1.0;
    }

    /** Helper to keep tracking the exact same tag ID inside a 4-tag HIVE cluster. */
    private FiducialResult getLockedTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            lockedTagId = -1;
            return null;
        }

        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            lockedTagId = -1;
            return null;
        }

        // Stick to our previously locked tag ID if it is still in view
        for (FiducialResult tag : tags) {
            if (tag.getFiducialId() == lockedTagId) {
                return tag;
            }
        }

        // Otherwise, lock onto the first visible tag in the cluster
        FiducialResult firstTag = tags.get(0);
        lockedTagId = firstTag.getFiducialId();
        return firstTag;
    }


    /**
     * Takes the current turntable position and returns the updated position.
     */
    public double update(double currentPosition) {
        LLResult result = limelight.getLatestResult();
        FiducialResult targetTag = getLockedTag();

        if (result != null && result.isValid() && targetTag != null) {
            lostTimer.reset();

            double timestamp = result.getTimestamp();
            if (timestamp != lastTimestamp) {
                lastTimestamp = timestamp;

                double tx = targetTag.getTargetXDegrees();
                double absTx = Math.abs(tx);

                // Hysteresis lock: stay dead still once inside STOP_TRACK_DEG
                if (isLocked) {
                    if (absTx > START_TRACK_DEG) {
                        isLocked = false;
                    }
                } else {
                    if (absTx <= STOP_TRACK_DEG) {
                        isLocked = true;
                    }
                }

                if (!isLocked) {
                    currentPosition += KP * tx;
                }
            }
            return currentPosition;
        }

        // No tag visible: reset lock and smoothly return to center after delay
        isLocked = false;
        lockedTagId = -1;

        if (lostTimer.seconds() > LOST_DELAY_SEC) {
            if (currentPosition > CENTER_POS + RETURN_STEP) {
                currentPosition -= RETURN_STEP;
            } else if (currentPosition < CENTER_POS - RETURN_STEP) {
                currentPosition += RETURN_STEP;
            } else {
                currentPosition = CENTER_POS;
            }
        }

        return currentPosition;
    }

    /**
     * Converts horizontal distance (inches) into a target flywheel velocity (ticks/sec)
     * tuned for the AndyMark NeveRest 1:1 Hex Motor (28 ticks/rev, ~2800 max ticks/sec).
     */
    public double getShooterVelocity(double distanceInches) {
        // Fallback cruising speed if no AprilTag is visible
        if (distanceInches < 0) {
            return 1900.0;
        }

        // Starting calibration points (adjust these two velocities after test shots!):
        double minDist = 36.0;    // Closest shooting spot (3 feet)
        double maxDist = 108.0;   // Farthest shooting spot (9 feet)
        double minVel  = 1500.0;  // Ticks/sec at 36 inches (~3,200 RPM)
        double maxVel  = 2500.0;  // Ticks/sec at 108 inches (~5,350 RPM)

        // Clamp distance so out-of-range readings stay safe
        double clampedDist = Math.max(minDist, Math.min(maxDist, distanceInches));

        // Linearly scale velocity between min and max distance
        double ratio = (clampedDist - minDist) / (maxDist - minDist);
        return minVel + ratio * (maxVel - minVel);
    }
}