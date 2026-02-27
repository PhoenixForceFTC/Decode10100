package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.DriveUtilsAdvanced;
import org.firstinspires.ftc.teamcode.utils.Location;
import org.firstinspires.ftc.teamcode.utils.RisingEdge;
//endregion

//region --- Controls ---
//----------------------------------------------------------------------
// Joystick 1 -----------------------------------------------------------
//  - Left Stick        - Mecanum Drive
//  - Right Stick       - Mecanum Rotate
//  - Left Stick Click  - Drive Speed High/Low
//  - Right Stick Click - Rotate Speed High/Low
//
//  - Dpad Up           - Move Forward (Slow)
//  - Dpad Down         - Move Back (Slow)
//  - Dpad Right        - Move Right (Slow)
//  - Dpad Left         - Move Left (Slow)
//
//  - Right Trigger     - Intake Spin In
//  - Right Bumpers     - Intake Spin Out
//  - Left Trigger      - Intake Arm Out
//  - Left Bumpers      - Intake Arm Back
//
//  - Y (▲)             - Next Step in Current Mode
//  - A (✕)             - Previous Step in Current Mode
//  - X (■)             - Intake In/Out
//  - B (○)             -
//
//----------------------------------------------------------------------
// Joystick 2 -----------------------------------------------------------
//  - Left Stick        -
//  - Right Stick       -
//  - Left Stick Click  - Reset Lift Encoder
//  - Right Stick Click - Reset Intake Encoder
//
//  - Dpad Up           - Manual Lift Up
//  - Dpad Down         - Manual Lift Down
//  - Dpad Right        - Manual Intake Out
//  - Dpad Left         - Manual Intake In
//
//  - Right Trigger     -
//  - Right Bumpers     -
//  - Left Trigger      -
//  - Left Bumpers      -

//  - Y (▲)             - Mode -> High Basket
//  - A (✕)             - Mode -> Low Basket
//  - X (■)             - Mode -> Climbing
//  - B (○)             - Mode -> Specimens
//----------------------------------------------------------------------
//endregion

@TeleOp(name="TeleOpOld RR", group="1")
public class TeleOp_Mecanum_RR extends LinearOpMode
{
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
    DriveUtilsAdvanced _driveUtilsAdvanced;
    public ElapsedTime _runtime = new ElapsedTime();


    enum position{
        Close,
        Medium,
        Far,
        None
    }
    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        //------------------------------------------------------------------------------------------
        //--- Robot Initialize
        //------------------------------------------------------------------------------------------
        int robotVersion = 1; //--- 1 for CRAB-IER and 2 for ARIEL


        _robot.init(robotVersion);
        _driveUtilsAdvanced = new DriveUtilsAdvanced(hardwareMap,new Pose2d(0,0,-3*Math.PI/4),_robot.drive,_robot.limelightHardware2Axis,this.telemetry,false);

        //_driveUtilsAdvanced = new DriveUtilsAdvanced(hardwareMap, Location.pose,_robot.drive,_robot.limelightHardware2Axis,this.telemetry);


        //------------------------------------------------------------------------------------------
        //--- Display and wait for the game to start (driver presses START)
        //------------------------------------------------------------------------------------------
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        _runtime.reset();


        //------------------------------------------------------------------------------------------
        //--- Hardware Initialize
        //------------------------------------------------------------------------------------------
        _robot.shooter.initialize();
       // _robot.intake.initialize();
        _robot.lights.initialize();

        //------------------------------------------------------------------------------------------
        //--- Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------
            //--- Hardware Run (updates lights, etc.)
            //------------------------------------------------------------------------------------------
            //_robot.run();
            _driveUtilsAdvanced.driveMecanum(gamepad1);
            _driveUtilsAdvanced.updateCamera();


            //------------------------------------------------------------------------------------------
            //--- Start Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + _runtime.toString());

            //------------------------------------------------------------------------------------------
            //--- Drive
            //------------------------------------------------------------------------------------------


            _robot.shooter.getTelemetry();

            //_robot.limelightHardware2Axis.loop();
            _robot.limelightHardware2Axis.servos();

            //------------------------------------------------------------------------------------------
            //--- Intake
            //------------------------------------------------------------------------------------------
           // _robot.intake.testColorSensors();  //--- Show color sensor values for tuning


            //------------------------------------------------------------------------------------------
            //--- Update Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.update();


        }
    }
}