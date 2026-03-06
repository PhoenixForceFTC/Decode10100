package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.MotifKicking;


import org.firstinspires.ftc.teamcode.hardware.Shooter;
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

@TeleOp(name="Red_TeleOp_State", group="1")
public class TeleOp_State_Red extends LinearOpMode
{
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
    DriveUtilsAdvanced _driveUtilsAdvanced;

    public ElapsedTime _runtime = new ElapsedTime();

    MotifKicking _kickMotif = new MotifKicking(_robot);


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
        int loop_count =0;
        int robotVersion = 1; //--- 1 for CRAB-IER and 2 for ARIEL
        int shooterSpeedRpm = 0;
        int shooterSpeedRpm3Ball = 0;
        boolean isThreeBallMode = false;
        boolean isAutoSpeed = true;
        boolean alreadyShot = false;
        position robotPosition = position.None;

        _robot.init(robotVersion);
        _driveUtilsAdvanced = new DriveUtilsAdvanced(hardwareMap,new Pose2d(0,0,-3*Math.PI/4),_robot.drive,_robot.limelightHardware2Axis,this.telemetry,false);

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
//        _robot.shooter.initialize();
//        // _robot.intake.initialize();
//        _robot.lights.initialize();

        //------------------------------------------------------------------------------------------
        //--- Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------
            //--- Hardware Run (updates lights, etc.)
            //------------------------------------------------------------------------------------------
            //_robot.run();
            _robot.intake.run();
            _robot.lights.run();

            if(_robot.kickers.runFinal((double) shooterSpeedRpm*  Shooter.ticksPerRotation/60,_robot.shooter.getSpeed(),true,(double) shooterSpeedRpm3Ball* Shooter.ticksPerRotation/60,-1)){
                _driveUtilsAdvanced.endAutoAlign();
                alreadyShot = false;
            }


            if(gamepad1.right_trigger>0.2){
                _driveUtilsAdvanced.autoAlign();
            }
            if(_driveUtilsAdvanced.driveMecanum(gamepad1,_robot.kickers)){
                if(!alreadyShot) {
                    if (isThreeBallMode) {
                        if (_robot.kickers.runFinal((double) shooterSpeedRpm * Shooter.ticksPerRotation / 60, _robot.shooter.getSpeed(), true, (double) shooterSpeedRpm3Ball * Shooter.ticksPerRotation / 60, 3)) {
                            _driveUtilsAdvanced.endAutoAlign();
                            alreadyShot = false;
                        }
                    } else {
                        _kickMotif.kickForMotifTeleOp();
                        _driveUtilsAdvanced.endAutoAlign();
                        alreadyShot = false;
                    }
                }
            }
            //_robot.driveRR.driveControl(1);

            if(gamepad1.aWasPressed()){
                _kickMotif.kickForMotifTeleOp();
                _driveUtilsAdvanced.endAutoAlign();
            }

            //------------------------------------------------------------------------------------------
            //--- Start Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + _runtime.toString());

            //------------------------------------------------------------------------------------------
            //--- Drive
            //------------------------------------------------------------------------------------------
            //_robot.drive.driveControl(0.5); //--- Both D-pad for directional movement and Joysticks for mecanum movement
            _driveUtilsAdvanced.updateCamera();
            if((loop_count % 20) == 0){
                _driveUtilsAdvanced.reset(true);
            }

            if(gamepad2.left_trigger > 0.7) {
                _robot.intake.forward();
            }

            if (gamepad2.right_trigger > 0.7){
                _robot.intake.backward();
            }

            if(gamepad2.left_bumper&&(shooterSpeedRpm>0)){
                if(shooterSpeedRpm>10){
                    shooterSpeedRpm -=10;
                    shooterSpeedRpm3Ball -=10;
                }
                else{
                    shooterSpeedRpm = 0;
                    shooterSpeedRpm3Ball = 0;
                }
            }
            if(gamepad2.right_bumper&&(shooterSpeedRpm<6000)){
                if(shooterSpeedRpm<5990){
                    shooterSpeedRpm +=10;
                    shooterSpeedRpm3Ball +=10;

                }
                else{
                    shooterSpeedRpm = 6000;
                    shooterSpeedRpm3Ball = 6000;
                }
            }
            //y close x mid a far and b toggle between 3 and 1

            if(!isThreeBallMode&&isAutoSpeed){
                shooterSpeedRpm=Math.round((float) ((_driveUtilsAdvanced.getDist()*10.1)+1630) );
            }
            if(gamepad1.x){
                isAutoSpeed=!isAutoSpeed;
            }
            if(gamepad2.y){
                robotPosition= position.Close;
                    shooterSpeedRpm3Ball = 2280;
                    shooterSpeedRpm = 2090;
            }
            if(gamepad2.x){
                robotPosition= position.Medium;
                    shooterSpeedRpm3Ball = 2650;
                    shooterSpeedRpm = 2430;
            }
            if(gamepad2.a){
                robotPosition= position.Far;
                    shooterSpeedRpm3Ball = 3330;
                    shooterSpeedRpm = 3060;
            }
            if(gamepad2.b){
                _robot.intake.stop();
            }
            //questionable
            if(g1RE.RisingEdgeButton(gamepad1, "y")){
                isThreeBallMode = !isThreeBallMode;
            }

            if(isThreeBallMode){
                _robot.shooter.shoot(shooterSpeedRpm3Ball);
            }
            else{
                _robot.shooter.shoot(shooterSpeedRpm);

            }


            telemetry.addData("target speed one ball in rpm", shooterSpeedRpm);
            telemetry.addData("target speed 3 ball in rpm", shooterSpeedRpm3Ball);
            telemetry.addData("three ball mode", isThreeBallMode);
            telemetry.addData("auto speed mode", isAutoSpeed);
            telemetry.addData("robot shooting position", robotPosition.toString());
            telemetry.addData("speed reading from the motor in ticks per second",_robot.shooter.getSpeed());
            _robot.shooter.getTelemetry();
            //_robot.limelightHardware.loop();
            _robot.limelightHardware2Axis.loop();
            _robot.limelightHardware2Axis.servos();

            //------------------------------------------------------------------------------------------
            //--- Intake
            //------------------------------------------------------------------------------------------
           // _robot.intake.testColorSensors();  //--- Show color sensor values for tuning


            //------------------------------------------------------------------------------------------
            //--- Update Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.addData("loop count", loop_count);
            loop_count++;
            telemetry.update();


        }
    }
}