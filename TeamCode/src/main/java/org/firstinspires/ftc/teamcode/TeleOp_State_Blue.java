package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "\uD83D\uDFE6Blue_TeleOp_State", group = "!State")
public class TeleOp_State_Blue extends TeleOp_State_Red {

    @Override
    public void runOpMode() {
        isBlue = true;  // override the variable before calling super
        super.runOpMode();  // runs all the base teleop logic
    }
}