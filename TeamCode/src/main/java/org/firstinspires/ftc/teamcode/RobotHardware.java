package org.firstinspires.ftc.teamcode;

//region --- Imports ---
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Intake_Incomplete;
import org.firstinspires.ftc.teamcode.hardware.Kickers;

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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashBoard = dashboard.getTelemetry();

    //------------------------------------------------------------------------------------------
    //--- Utility Motors
    //------------------------------------------------------------------------------------------
    public DcMotorEx motorShooterLeft = null;
    public DcMotorEx motorShooterRight = null;
    public DcMotorEx motorIntake = null;

    //------------------------------------------------------------------------------------------
    //--- Servos
    //------------------------------------------------------------------------------------------
    public Servo kicker1 = null;
    public Servo kicker2 = null;
    public Servo kicker3 = null;

    //------------------------------------------------------------------------------------------
    //--- Custom Hardware Classes
    //------------------------------------------------------------------------------------------
    public Shooter shooter;
    public Intake_Incomplete intake;
    public LimelightHardware limelightHardware;
    public Drive drive;
    public IMU imu;
    public Kickers kickers;

    public Limelight3A limelight;
    public RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
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
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //myOpMode.telemetry = dashboard.getTelemetry();
        // i do this in the main teleop now that might be wrong code structure

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
        motorIntake = myOpMode.hardwareMap.get(DcMotorEx.class, "intake_motor");

        motorShooterLeft.setDirection(DcMotor.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotor.Direction.FORWARD);
        motorIntake.setDirection(DcMotor.Direction.FORWARD);



        motorShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorShooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker1 = myOpMode.hardwareMap.get(Servo.class, "kicker_1");
        kicker2 = myOpMode.hardwareMap.get(Servo.class, "kicker_2");
        kicker3 = myOpMode.hardwareMap.get(Servo.class, "kicker_3");

        imu = myOpMode.hardwareMap.get(IMU.class,"imu");
        limelight = myOpMode.hardwareMap.get(Limelight3A.class,"limelight");
        //imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


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
        intake = new Intake_Incomplete(
                motorIntake,
                myOpMode.gamepad2,
                myOpMode.telemetry,
                false);

        limelightHardware = new LimelightHardware(imu,
                limelight,
                myOpMode.gamepad1,
                myOpMode.telemetry,
                1,
                true);

        kickers = new Kickers(kicker1,
                kicker2,
                kicker3,
                myOpMode.gamepad2,
                myOpMode.telemetry,
                true);


        //------------------------------------------------------------------------------------------
        //--- Messages
        //------------------------------------------------------------------------------------------
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
}