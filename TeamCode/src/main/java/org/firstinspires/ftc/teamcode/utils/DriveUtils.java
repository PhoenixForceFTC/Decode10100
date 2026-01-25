package org.firstinspires.ftc.teamcode.utils;

//region --- Imports ---
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

@Config
public class DriveUtils
{
    public static double FrontMultiplier = 1.2;
    //--- Arcade Drive Method
    public static void arcadeDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight,
                                   Gamepad gamepad, Telemetry telemetry, boolean showInfo,
                                   double speedMultiplier, double speedMultiplierRotate)
    {
        double max;

        //--- POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad.left_stick_y;  //--- Note, pushing stick forward gives negative value
        double lateral = gamepad.left_stick_x;

        double yaw = (gamepad.right_stick_x)*Math.abs(gamepad.right_stick_x) * speedMultiplierRotate; //--- Scale yaw separately

        //--- Combine the joystick requests for each axis-motion to determine each wheel's power.
        double leftFrontPower = (axial + lateral + yaw) * speedMultiplier* FrontMultiplier;
        double rightFrontPower = (axial - lateral - yaw) * speedMultiplier* FrontMultiplier;
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
    public static void arcadeDrive2(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight,
                                   double lateral, double axial, double yaw, double yawImportant, Telemetry telemetry, boolean showInfo,
                                   double speedMultiplier, double speedMultiplierRotate)
    {
        double max;

        //--- POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double _axial = axial;  //--- Note, pushing stick forward gives negative value
        double _lateral = lateral;

        double _yaw = ((yaw)*Math.abs(yaw) * speedMultiplierRotate)+yawImportant; //--- Scale yaw separately

        //--- Combine the joystick requests for each axis-motion to determine each wheel's power.
        double leftFrontPower = (_axial + _lateral + _yaw) * speedMultiplier* FrontMultiplier;
        double rightFrontPower = (_axial - _lateral - _yaw) * speedMultiplier* FrontMultiplier;
        double leftBackPower = (_axial - _lateral + _yaw) * speedMultiplier;
        double rightBackPower = (_axial + _lateral - _yaw) * speedMultiplier;

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
            telemetry.addData("Control -> Axial/Lateral/Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, _yaw);
            telemetry.addData("Motor -> Front Left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Motor -> Back Left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        }
    }
}