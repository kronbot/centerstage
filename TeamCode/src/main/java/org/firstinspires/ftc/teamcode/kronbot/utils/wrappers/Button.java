package org.firstinspires.ftc.teamcode.kronbot.utils.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A wrapper for a button that allows for toggling, short presses, and long presses.
 *
 * @version 1.0
 */
@Config
public class Button {
    boolean buttonValue = false;
    boolean lockedShort = false;
    boolean lockedLong = false;
    boolean lockedToggle = false;

    public void updateButton(boolean input) {
        buttonValue = input;
    }

    public boolean press() {
        return buttonValue;
    }

    boolean lastIteration = false;
    boolean currentIteration = false;
    boolean toggleStatus = false;

    public boolean toggle() {
        lastIteration = currentIteration;
        currentIteration = buttonValue;

        if (!lastIteration && currentIteration) {
            if (!lockedToggle) {
                toggleStatus = !toggleStatus;
                lockedToggle = true;
            }
            return true;

        } else {
            lockedToggle = false;
        }

        return false;
    }

    public static double longPressTime = 1000;
    boolean longPressLastIteration = false;
    boolean longPressCurrentIteration = false;
    boolean longToggle = false;
    ElapsedTime longPressTimer = new ElapsedTime();

    public boolean longPress() {
        longPressLastIteration = longPressCurrentIteration;
        longPressCurrentIteration = buttonValue;

        if (!longPressLastIteration && longPressCurrentIteration) {
            longPressTimer.reset();
        }

        if (!longPressCurrentIteration) {
            longPressTimer.reset();
        }

        if (longPressLastIteration && longPressCurrentIteration
            && longPressTimer.milliseconds() > longPressTime) {
            if (!lockedLong) {
                longToggle = !longToggle;
                lockedLong = true;
                return true;
            }
            return false;
        } else {
            lockedLong = false;
        }

        return false;
    }

    boolean shortLastIteration = false;
    boolean shortCurrentIteration = false;
    boolean shortToggle = false;
    ElapsedTime shortTimer = new ElapsedTime();
    public static double shortPressTime = 500;

    public boolean shortPress() {
        shortLastIteration = shortCurrentIteration;
        shortCurrentIteration = buttonValue;

        if (!shortLastIteration && shortCurrentIteration) {
            shortTimer.reset();
        }
        if (shortLastIteration && !shortCurrentIteration
            && shortTimer.milliseconds() < shortPressTime) {
            shortTimer.reset();
            if (!lockedShort) {
                shortToggle = !shortToggle;
                lockedShort = true;
            }
            return true;
        } else {
            lockedShort = false;
        }

        return false;
    }

    public boolean getToggleStatus() {
        return toggleStatus;
    }

    public boolean getShortToggle() {
        return shortToggle;
    }

    public boolean getLongToggle() {
        return longToggle;
    }

    public void setToggleStatus(boolean status) {
        toggleStatus = status;
    }

    public void setShortToggle(boolean status) {
        shortToggle = status;
    }

    public void setLongToggle(boolean status) {
        longToggle = status;
    }

    public void resetToggles() {
        toggleStatus = false;
        shortToggle = false;
        longToggle = false;
    }

    public void setLongPressTime(double milliseconds) {
        longPressTime = milliseconds;
    }

    public void setShortPressTime(double milliseconds) {
        shortPressTime = milliseconds;
    }
}
