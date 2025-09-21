package org.firstinspires.ftc.teamcode.utils;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

public class DriveUtils
{
    //--- Arcade Drive Method
    public static void arcadeDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight,
                                   Gamepad gamepad, Telemetry telemetry, boolean showInfo,
                                   double speedMultiplier, double speedMultiplierRotate)
    {
        double max;

        //--- POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad.left_stick_y;  //--- Note, pushing stick forward gives negative value
        double lateral = gamepad.left_stick_x;
        double yaw = gamepad.right_stick_x * speedMultiplierRotate; //--- Scale yaw separately

        //--- Combine the joystick requests for each axis-motion to determine each wheel's power.
        double leftFrontPower = (axial + lateral + yaw) * speedMultiplier;
        double rightFrontPower = (axial - lateral - yaw) * speedMultiplier;
        double leftBackPower = (axial - lateral + yaw) * speedMultiplier;
        double rightBackPower = (axial + lateral - yaw) * speedMultiplier;

        //--- Normalize the values so no wheel power exceeds 100%
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0)
        {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        //--- Send calculated power to wheels
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        rearLeft.setPower(leftBackPower);
        rearRight.setPower(rightBackPower);

        //--- Show telemetry if enabled
        if (showInfo)
        {
            telemetry.addData("Control -> Axial/Lateral/Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
            telemetry.addData("Motor -> Front Left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Motor -> Back Left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        }
    }
}