package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class RisingEdge
{
    // Per-button previous state — avoids the full-gamepad copy that broke multi-button detection.
    private final HashMap<String, Boolean> _prevState = new HashMap<>();

    public RisingEdge() {}

    /**
     * Returns true on the first loop that the named button transitions from not-pressed to pressed.
     * Safe to call for multiple buttons in the same loop — each button is tracked independently.
     */
    public boolean RisingEdgeButton(Gamepad gamepad, String button) {
        boolean current = getButton(gamepad, button);
        boolean previous = _prevState.containsKey(button) && _prevState.get(button);
        _prevState.put(button, current);
        return current && !previous;
    }

    private boolean getButton(Gamepad g, String button) {
        switch (button) {
            case "a": return g.a;
            case "b": return g.b;
            case "x": return g.x;
            case "y": return g.y;

            case "left_bumper":  return g.left_bumper;
            case "right_bumper": return g.right_bumper;

            case "dpad_up":    return g.dpad_up;
            case "dpad_down":  return g.dpad_down;
            case "dpad_left":  return g.dpad_left;
            case "dpad_right": return g.dpad_right;

            default: return false;
        }
    }
}
