package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
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
@Disabled
@TeleOp(name="TeleOp_Mecanum_New", group="1")
public class TeleOp_Mecanum_New extends LinearOpMode
{
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
    public ElapsedTime _runtime = new ElapsedTime();
    public static Pose2d lastAutoPose = new Pose2d(0, 0, 0);




    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    @Override

    public void runOpMode()
    {
        //------------------------------------------------------------------------------------------
        //--- Robot Initialize
        //------------------------------------------------------------------------------------------
        int robotVersion = 1; //--- 1 for Alpha and 2 for Beta
        int speed = 0;
        boolean isShooting = false;
        boolean readyForShooting = false;
        boolean trippleShooting = false;
        Double floorDistance = null;

        ElapsedTime servoScanTimer = new ElapsedTime();
        double targetPitch = 0.6;

        _robot.init(robotVersion);

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
            telemetry.addData("floor distance final", floorDistance);
            telemetry.addData("target pitch", targetPitch);
            telemetry.addData("is shooting", isShooting);
            telemetry.addData("ready for shooting", readyForShooting);
            // Limelight aiming and shooting logic
            // Right trigger ENTERS shooting mode
            if(gamepad1.right_trigger > 0.7){
                if (!isShooting) { // Runs only on the first press of the trigger
                    isShooting = true;
                    readyForShooting = false; // Assume not ready until a target is found
                    servoScanTimer.reset(); // Start the scan timer
                }
            }

            // Left trigger EXITS shooting mode
            if (gamepad1.left_trigger > 0.7) {
                if (isShooting) {
                    // Exit shooting mode
                    isShooting = false;
                    readyForShooting = false;
                    floorDistance = null; // Reset distance when shooting stops
                    _robot.limelightHardware2Axis.setServos(0.5, 0.6); // Reset servos to default
                    targetPitch = 0.6;
                }
            }

            // Handle servo scanning when trying to shoot but no target is locked
            if (isShooting && !readyForShooting) {
                // Try to find a target.
                floorDistance = _robot.limelightHardware2Axis.getFloorDistance();

                if (floorDistance != null) {
                    // Target found and locked
                    readyForShooting = true;
                    // You might want to lock the servo here, e.g., _robot.limelightHardware2Axis.setServos(0.5, currentPitch);
                } else {
                    // No target found, continue scanning
                    if (servoScanTimer.seconds() > 0.7) {
                        // Every 0.2 seconds, swap the target pitch
                        if (targetPitch == 0.5) targetPitch = 0.6;
                        else targetPitch = 0.5;
                        _robot.limelightHardware2Axis.setServos(0.5, targetPitch);
                        servoScanTimer.reset();
                    }
                }
            }


            //------------------------------------------------------------------------------------------
            //--- Start Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + _runtime.toString());
            //------------------------------------------------------------------------------------------
            //--- Camera
            //------------------------------------------------------------------------------------------
            _robot.limelightHardware2Axis.loop();
            _robot.limelightHardware2Axis.servos();
            //------------------------------------------------------------------------------------------
            //--- Drive
            //------------------------------------------------------------------------------------------
            _robot.drive.driveControl(0.5); //--- Both D-pad for directional movement and Joysticks for mecanum movement
            //------------------------------------------------------------------------------------------
            //--- Intake
            //------------------------------------------------------------------------------------------
            _robot.intake.run();




            //------------------------------------------------------------------------------------------
            //--- Shooter
            //------------------------------------------------------------------------------------------
            _robot.shooter.shoot(speed);//change so speed is set in shooter based on the distance
            telemetry.addData("speed reading from the motor in ticks per second",_robot.shooter.getSpeed());
            _robot.shooter.getTelemetry();



            //_robot.arm.controlArmManual();
            if(gamepad2.left_bumper&&speed>0){
                if(speed>10){
                    speed -=10;
                }
                else{
                    speed = 0;
                }
            }
            if(gamepad2.right_bumper&&speed<6000){
                if(speed<5990){
                    speed +=10;

                }
                else{
                    speed = 6000;
                }
            }
            _robot.kickers.run(_robot.shooter.speed, _robot.shooter.getSpeed(),readyForShooting);
            //------------------------------------------------------------------------------------------
            //--- Update Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.update();

        }
    }
}
