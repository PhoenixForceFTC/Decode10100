package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.DriveUtilsAdvanced;
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

@TeleOp(name="TeleOpOldMecanumTest", group="1")
public class TeleOp_Mecanum_test extends LinearOpMode
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
        int shooterSpeedRpm = 0;
        boolean isThreeBallMode = false;
        position robotPosition = position.None;

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
       // _robot.intake.initialize();
        _robot.lights.initialize();
        _robot.limelightHardware2Axis.servos();
        _robot.kickers.initialize();
       // _robot.intake.initialize();

        //------------------------------------------------------------------------------------------
        //--- Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------
            //--- Hardware Run (updates lights, etc.)
            //------------------------------------------------------------------------------------------

            //------------------------------------------------------------------------------------------
            //--- Start Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + _runtime.toString());

            //------------------------------------------------------------------------------------------
            //--- Drive
            //------------------------------------------------------------------------------------------
           // _robot.drive.driveControl(0.5); //--- Both D-pad for directional movement and Joysticks for mecanum movement

            _robot.limelightHardware2Axis.loop();
            if (gamepad1.a) { // hold 'A' to auto-align
                if (_robot.limelightHardware2Axis.hasValidTarget()) {
                    double tx = _robot.limelightHardware2Axis.getTx();
                    double turnPower = 0.02 * tx;
                    if (Math.abs(tx) < 0.1) turnPower = 0;

                    // Rotate only
                    _robot.driveRR.mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), -turnPower));

                    telemetry.addData("Tx", tx);
                    telemetry.addData("Turn Power", -turnPower);
                } else {
                    _robot.driveRR.mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
                    telemetry.addLine("No target detected");
                }
            } else {
                // normal manual control
                _robot.driveRR.driveControl(1.0);
            }

            _robot.driveRR.mecanumDrive.updatePoseEstimate();

            Pose2d pose = _robot.driveRR.getPosition();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading", pose.heading);
            telemetry.update();


            if (gamepad2.b){
                _robot.shooter.shoot(0);
            }

            if(gamepad2.left_trigger > 0.7) {
                _robot.intake.forward();
            }

            if (gamepad2.right_trigger > 0.7){
                _robot.intake.backward();
            }
            if (gamepad2.y){
                shooterSpeedRpm = 2090;
            }
            if (gamepad2.x){
                shooterSpeedRpm = 2430;
            }
            if (gamepad2.a){
                shooterSpeedRpm = 3060;
            }

            //shooterSpeedRpm=Math.round((float) ((_driveUtilsAdvanced.getDist()*10.1)+1630) );

            _robot.kickers.run(_robot.shooter.speed,_robot.shooter.getSpeed(),true);
            _robot.intake.run();
            _robot.shooter.shoot(shooterSpeedRpm);


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