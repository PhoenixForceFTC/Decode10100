package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name="3ptAuto")
public class ThreePointAuto extends LinearOpMode {
    private MecanumWheels MecanumWheels;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode(){
        MecanumWheels = new MecanumWheels(this);

        waitForStart();

        timer.reset();
        while(timer.time()<=2){
            MecanumWheels.move(0, 1, 0);
        }
    }
}
