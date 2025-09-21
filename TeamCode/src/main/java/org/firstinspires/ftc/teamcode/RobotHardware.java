package org.firstinspires.ftc.teamcode;

//region --- Imports ---
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Shooter;

import org.firstinspires.ftc.teamcode.hardware.Drive;
//endregion

//region --- Control Hub Config ---
/*
Motor
0 - upl (Lifting Arm Left) + Encoder
1 - upr (Lifting Arm Righ) + Encoder
2 - fr (Front Right)
3 - fl (Front Left)

Servo
0 - in-lspin (Intake Left Spinner)
1 - ????? (not working)
2 - in-rspin (Intake Right Spinner)
3 - in-lup (Intake Left Up/Down)
4 - in-rup (Intake Right Up/Down)
5 - claw
*/
//endregion

//region --- Expansion Hub Config ---
/*
Motor
0 - rl (Rear Left)
1 - rr (Rear Right)
2 - in-arm (Intake Extension Arm)
3 -

Servo
0 - shoulder-r
1 - elbow
2 - wrist
3 - ????? (not working)
4 - ????? (not working)
5 - shoulder-l

I2C
0 - color
1 -
2 - camera
3 - odo (odometry)
*/
//endregion

public class RobotHardware {

    //------------------------------------------------------------------------------------------
    //--- Settings
    //------------------------------------------------------------------------------------------
    boolean _showInfo = true;

    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //------------------------------------------------------------------------------------------
    //--- Drive Motors
    //------------------------------------------------------------------------------------------
    public DcMotor motorDriveFrontRight = null;
    public DcMotor motorDriveFrontLeft = null;
    public DcMotor motorDriveRearRight = null;
    public DcMotor motorDriveRearLeft = null;

    //------------------------------------------------------------------------------------------
    //--- Utility Motors
    //------------------------------------------------------------------------------------------
    public DcMotorEx motorShooterLeft = null;
    public DcMotorEx motorShooterRight = null;

    //------------------------------------------------------------------------------------------
    //--- Servos
    //------------------------------------------------------------------------------------------


    //------------------------------------------------------------------------------------------
    //--- Custom Hardware Classes
    //------------------------------------------------------------------------------------------
    public Shooter shooter;
    public Drive drive;

    //------------------------------------------------------------------------------------------
    //--- Define a constructor that allows the OpMode to pass a reference to itself
    //------------------------------------------------------------------------------------------
    public RobotHardware(LinearOpMode opmode)
    {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init(int robotVersion)
    {
        //------------------------------------------------------------------------------------------
        //--- Motor Config
        //------------------------------------------------------------------------------------------
        //--- Drive Motors
        motorDriveFrontLeft = myOpMode.hardwareMap.get(DcMotor.class, "fl");
        motorDriveRearLeft = myOpMode.hardwareMap.get(DcMotor.class, "rl");
        motorDriveFrontRight = myOpMode.hardwareMap.get(DcMotor.class, "fr");
        motorDriveRearRight = myOpMode.hardwareMap.get(DcMotor.class, "rr");

        //--- Drive Motor Directions
        motorDriveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRearLeft.setDirection(DcMotor.Direction.FORWARD);
        motorDriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRearRight.setDirection(DcMotor.Direction.REVERSE);

        //--- Utility Motors
        motorShooterLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "left_launch_top"); //--- Lifting Arm Left
        motorShooterRight = myOpMode.hardwareMap.get(DcMotorEx.class, "right_launch_top"); //--- Lifting Arm Right

        motorShooterLeft.setDirection(DcMotor.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotor.Direction.FORWARD);



        motorShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorShooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //------------------------------------------------------------------------------------------
        //--- Servo Config
        //------------------------------------------------------------------------------------------
        //--- Servos (Continuous)

        //--- Configure Servos
        //servoArmWrist.setDirection(Servo.Direction.REVERSE);

        //------------------------------------------------------------------------------------------
        //--- Hardware Constructors
        //------------------------------------------------------------------------------------------
        shooter = new Shooter(
                motorShooterLeft,
                motorShooterRight,
                myOpMode.gamepad1,
                myOpMode.telemetry,
                _showInfo
        );

        drive = new Drive(
                motorDriveFrontLeft,
                motorDriveFrontRight,
                motorDriveRearLeft,
                motorDriveRearRight,
                myOpMode.gamepad1,
                myOpMode.telemetry,
                robotVersion,
                _showInfo
        );
        //------------------------------------------------------------------------------------------
        //--- Messages
        //------------------------------------------------------------------------------------------
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
}