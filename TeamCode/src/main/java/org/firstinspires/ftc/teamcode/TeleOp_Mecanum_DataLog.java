package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.DataLog;
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

@TeleOp(name="TeleOpOld", group="1")
public class TeleOp_Mecanum_DataLog extends LinearOpMode
{
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
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
        int shooterSpeedRpm = 0;
        boolean isThreeBallMode = false;
        position robotPosition = position.None;
        DataLog currentData;

        _robot.init(robotVersion);
        RisingEdge g1RE = new RisingEdge();

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

        //------------------------------------------------------------------------------------------
        //--- Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------
            //--- Start Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + _runtime.toString());

            //------------------------------------------------------------------------------------------
            //--- Drive
            //------------------------------------------------------------------------------------------
            _robot.drive.driveControl(0.5); //--- Both D-pad for directional movement and Joysticks for mecanum movement


            if(gamepad2.left_bumper&&(shooterSpeedRpm>0)){
                if(shooterSpeedRpm>10){
                    shooterSpeedRpm -=10;
                }
                else{
                    shooterSpeedRpm = 0;
                }
            }
            if(gamepad2.right_bumper&&(shooterSpeedRpm<6000)){
                if(shooterSpeedRpm<5990){
                    shooterSpeedRpm +=10;

                }
                else{
                    shooterSpeedRpm = 6000;
                }
            }
            //y close x mid a far and b toggle between 3 and 1
            if(gamepad2.y){
                robotPosition= position.Close;
                if (isThreeBallMode) {
                    shooterSpeedRpm = 2280;
                } else {
                    shooterSpeedRpm = 2090;
                }
            };
            if(gamepad2.x){
                robotPosition= position.Medium;
                if (isThreeBallMode) {
                    shooterSpeedRpm = 2650;
                } else {
                    shooterSpeedRpm = 2430;
                }
            };
            if(gamepad2.a){
                robotPosition= position.Far;
                if (isThreeBallMode) {
                    shooterSpeedRpm = 3330;
                } else {
                    shooterSpeedRpm = 3060;
                }
            };
            if(g1RE.RisingEdgeButton(gamepad2, "b")){
                isThreeBallMode = !isThreeBallMode;
            }
            _robot.shooter.shoot(shooterSpeedRpm);
            _robot.intake.run();
            DataLog.Shooter shooterPos =_robot.kickers.run2(_robot.shooter.speed,_robot.shooter.getSpeed(),true);
            if(shooterPos != DataLog.Shooter.Unkown){

                currentData = new DataLog(0,0,0,0,0,false,0,0,0,0,shooterPos);

            }
            if(gamepad1.right_bumper){

            }

            telemetry.addData("target speed in rpm", shooterSpeedRpm);
            telemetry.addData("three ball mode", isThreeBallMode);
            telemetry.addData("robot shooting position", robotPosition.toString());
            telemetry.addData("speed reading from the motor in ticks per second",_robot.shooter.getSpeed());
            _robot.shooter.getTelemetry();
            //_robot.limelightHardware.loop();
            _robot.limelightHardware2Axis.loop();
            _robot.limelightHardware2Axis.servos();

            //------------------------------------------------------------------------------------------
            //--- Update Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.update();
        }
    }
}