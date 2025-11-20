package org.firstinspires.ftc.teamcode.utils;

//region --- Imports ---
//endregion

import com.qualcomm.robotcore.hardware.Gamepad;

public class RisingEdge
{
    public Gamepad previous = null;

    public boolean RisingEdgeButton(Gamepad gamepad, String button) {
        if(previous!=null) {
            if (getButton(gamepad, button) != getButton(previous, button) && getButton(gamepad, button)) {
                return true;
            }
        }
        this.previous=gamepad;
        return false;
    }
    private boolean getButton(Gamepad g, String button) {
        switch (button) {
            case "a": return g.a;
            case "b": return g.b;
            case "x": return g.x;
            case "y": return g.y;

            case "left_bumper": return g.left_bumper;
            case "right_bumper": return g.right_bumper;

            case "dpad_up": return g.dpad_up;
            case "dpad_down": return g.dpad_down;
            case "dpad_left": return g.dpad_left;
            case "dpad_right": return g.dpad_right;

            default: return false;
        }
    }
}