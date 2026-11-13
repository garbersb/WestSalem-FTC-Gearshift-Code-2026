package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *   NOT_VISIBLE : no tag for more than LOST_DELAY_SEC (driver needs to move the robot)
 *   TRACKING    : tag visible, but not aimed or not in shooting range
 *   READY       : tag visible, |tx| within tolerance and ty within tested range
 *
 * This class only decides the state, we dont any actuating
 */
public class StateMachine {

    public enum State { NOT_VISIBLE, TRACKING, READY }

    // ---- We need to test and tune these ----
    private static final double LOST_DELAY_SEC = 0.5; // no tag for this long = NOT_VISIBLE
    private static final double MAX_ABS_TX = 2.0;     // when turntable is turned straight towards the tag, this should be 0, im giving it 2 degrees of allowed error
    private static final double MIN_TY = -5.0;        // TODO: replace with tested value (closest shot)
    private static final double MAX_TY = 5.0;         // TODO: replace with tested value (farthest shot)

    private State state = State.NOT_VISIBLE;
    private final ElapsedTime lostTimer = new ElapsedTime();

    public State update(boolean tagVisible, double tx, double ty) {
        if (tagVisible) {
            lostTimer.reset();
        }
        boolean tagLost = lostTimer.seconds() > LOST_DELAY_SEC;

        boolean withinRange = Math.abs(tx) <= MAX_ABS_TX
                && ty > MIN_TY && ty < MAX_TY;

        switch (state) {
            case NOT_VISIBLE:
                if (tagVisible) {
                    state = State.TRACKING;
                }
                break;

            case TRACKING:
                if (tagLost) {
                    state = State.NOT_VISIBLE;
                } else if (tagVisible && withinRange) {
                    state = State.READY;
                }
                break;

            case READY:
                if (tagLost) {
                    state = State.NOT_VISIBLE;
                } else if (tagVisible && !withinRange) {
                    state = State.TRACKING;
                }
                break;
        }
        return state;
    }

    public State getState() { return state; }
}