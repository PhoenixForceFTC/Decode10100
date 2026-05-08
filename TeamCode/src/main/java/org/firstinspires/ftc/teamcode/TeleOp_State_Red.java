package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Lights;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.hardware.MotifKicking;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.utils.DriveUtilsAdvanced;
import org.firstinspires.ftc.teamcode.utils.Location;
import org.firstinspires.ftc.teamcode.utils.RisingEdge;
//endregion

//region --- Controls ---
//----------------------------------------------------------------------
// Joystick 1 -----------------------------------------------------------
//  - Left Stick        - Mecanum Drive
//  - Right Stick       - Mecanum Rotate
//
//  - Dpad Up           - Move Forward (Slow)
//  - Dpad Down         - Move Back (Slow)
//  - Dpad Right        - Rotate Right (Slow)
//  - Dpad Left         - Rotate Left (Slow)
//
//  - Right Trigger     - Hold to auto-align to goal; release to shoot
//  - Left Bumper       - (hold) Override: fire all 3 via runFinal
//
//  - Y (▲)             - Toggle 3-ball / 1-ball mode
//  - A (✕)             - Fire motif kick sequence (non-blocking)
//  - X (■)             - Toggle auto-speed
//  - B (○)             - Kick middle position (manual override)
//
//----------------------------------------------------------------------
// Joystick 2 -----------------------------------------------------------
//  - Left Stick        -
//  - Right Stick       -
//  - Left Stick Click  - Cancel auto-align
//  - Right Stick Click - Toggle kickstand (0 deg OFF / 90 deg ON)
//
//  - Dpad Left         - Manual kick left  (requires shooter at speed)
//  - Dpad Up           - Manual kick middle (requires shooter at speed)
//  - Dpad Right        - Manual kick right  (requires shooter at speed)
//  - Dpad Down         - Manual kick all 3  (3-ball, requires speed)
//
//  - Right Trigger     - Outtake (forward)
//  - Left Trigger      - Intake (backward)
//  - Right Bumper      - Shooter RPM +10
//  - Left Bumper       - Shooter RPM -10
//
//  - B (○)             - Stop intake
//  - Y (▲)             - Speed preset: Close  (2090 / 2280 RPM)
//  - X (■)             - Speed preset: Medium (2430 / 2650 RPM)
//  - A (✕)             - Speed preset: Far    (3060 / 3330 RPM)
//----------------------------------------------------------------------
//endregion

@Config
@TeleOp(name="\uD83D\uDFE5Red_TeleOp_State", group = "!State")
public class TeleOp_State_Red extends LinearOpMode {

    // --- Speed tuning (live-editable via FTC Dashboard) ---
    // 3-ball simultaneous: all 3 fly at once, higher RPM needed
    public static int SPEED_3BALL_CLOSE  = 2280;
    public static int SPEED_3BALL_MEDIUM = 2650;
    public static int SPEED_3BALL_FAR    = 3330;
    // 1-ball sequential: one at a time, lower RPM is fine
    public static int SPEED_1BALL_CLOSE  = 2090;
    public static int SPEED_1BALL_MEDIUM = 2430;
    public static int SPEED_1BALL_FAR    = 3060;
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
    Boolean isBlue = false;
    DriveUtilsAdvanced _driveUtilsAdvanced;

    public ElapsedTime _runtime = new ElapsedTime();

    MotifKicking _kickMotif = new MotifKicking(_robot);

    public boolean overrideBallDistanceDetection = false;
    public ElapsedTime overrideTimer = new ElapsedTime(3);


    enum position {
        Close,
        Medium,
        Far,
        None
    }

    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        //------------------------------------------------------------------------------------------
        //--- Robot Initialize
        //------------------------------------------------------------------------------------------
        int loop_count = 0;
        int robotVersion = 1;
        int shooterSpeedRpm = 0;
        int shooterSpeedRpm3Ball = 0;
        boolean isThreeBallMode = true;
        boolean isAutoSpeed = true;

        position robotPosition = position.None;

        // Trigger-release auto-align state
        boolean wasAlignTriggered = false;

        // Limelight motif detection (update MotifKicking.GameMotif once per match)
        boolean motifUpdatedFromLL = false;

        // 3-ball vibration rising-edge detection
        boolean prev3Balls = false;

        // Kick-completion detection: restart intake when sequence finishes
        boolean wasKicking = false;

        _robot.init(robotVersion);
        telemetry.addData("location string in teleopState", Location.GetPose());
        telemetry.addData("Class Hash in teleopState", Location.class.hashCode());
        _driveUtilsAdvanced = new DriveUtilsAdvanced(hardwareMap, Location.pose, _robot.drive,
                _robot.limelightHardware2Axis, this.telemetry, isBlue, _robot);

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
        _robot.lights.initialize();

        // Switch to MANUAL bulk caching for tightest loop timing — clearBulkCache() is
        // called at the top of every loop iteration below. Autos stay on AUTO (set in init()).
        _robot.enableManualBulkReads();

        //------------------------------------------------------------------------------------------
        //--- Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {
            loop_count++;

            // Refresh the I2C bulk-read cache once per loop.
            _robot.clearBulkCache();

            // Cache shooter velocity once; all getSpeed() calls this loop reuse it.
            _robot.shooter.cacheVelocity();
            double shooterSpeed = _robot.shooter.getSpeed();

            telemetry.addData("Run Time", " " + _runtime.toString() + " Loop Count:" + loop_count);

            //------------------------------------------------------------------------------------------
            //--- Lights blink state machine
            //------------------------------------------------------------------------------------------
            _robot.lights.run();

            // Lock intake's updateLights() when aligning so red override wins.
            // Must be set before _robot.run() / intake.run() calls below.
            _robot.intake.lightsLocked = _driveUtilsAdvanced.isAligning;

            //------------------------------------------------------------------------------------------
            //--- Kickers: G2 dpad manual override always active via runFinal()
            //------------------------------------------------------------------------------------------
            if (!gamepad1.left_bumper) {
                if (_robot.kickers.runFinal((double) shooterSpeedRpm * Shooter.ticksPerRotation / 60,
                        shooterSpeed, true,
                        (double) shooterSpeedRpm3Ball * Shooter.ticksPerRotation / 60,
                        -1, _robot.intake)) {
                    _driveUtilsAdvanced.endAutoAlign();
                }
            } else {
                if (_robot.kickers.runFinal((double) shooterSpeedRpm * Shooter.ticksPerRotation / 60,
                        shooterSpeed, true,
                        (double) shooterSpeedRpm3Ball * Shooter.ticksPerRotation / 60,
                        3, _robot.intake)) {
                    _driveUtilsAdvanced.endAutoAlign();
                }
            }

            _robot.run();

            //------------------------------------------------------------------------------------------
            //--- G1 A: fire kick sequence (non-blocking)
            //    3-ball mode = simultaneous, 1-ball mode = sequential motif order
            //------------------------------------------------------------------------------------------
            if (gamepad1.a) {
                _kickMotif.setKick(isThreeBallMode);
            }
            _kickMotif.checkKick();

            // Falling-edge: kick sequence just finished → clear ball detection so auto-intake restarts
            boolean isKickingNow = _kickMotif.isKicking();
            if (wasKicking && !isKickingNow) {
                _robot.intake.clearAllSensorValues();
            }
            wasKicking = isKickingNow;

            //------------------------------------------------------------------------------------------
            //--- Limelight motif detection: update GameMotif on first obelisk tag seen
            //------------------------------------------------------------------------------------------
            if (!motifUpdatedFromLL) {
                LimelightHardware2Axis.Motif llMotif = _robot.limelightHardware2Axis.storedGameMotif;
                if (llMotif != null) {
                    switch (llMotif) {
                        case GPP: MotifKicking.GameMotif = MotifKicking.Motif.GPP; break;
                        case PGP: MotifKicking.GameMotif = MotifKicking.Motif.PGP; break;
                        case PPG: MotifKicking.GameMotif = MotifKicking.Motif.PPG; break;
                    }
                    motifUpdatedFromLL = true;
                }
            }

            //------------------------------------------------------------------------------------------
            //--- Kickstand: G2 right stick click toggles 0 deg / 90 deg
            //------------------------------------------------------------------------------------------
            _robot.kickstand.run();

            //------------------------------------------------------------------------------------------
            //--- Camera & Localizer
            //------------------------------------------------------------------------------------------
            _driveUtilsAdvanced.updateCameraPitch();
            _robot.limelightHardware2Axis.loop();
            _robot.limelightHardware2Axis.servos();
            if ((loop_count % 20) == 0) {
                _driveUtilsAdvanced.reset(true);
            }

            //------------------------------------------------------------------------------------------
            //--- Trigger-release auto-align shooting state machine
            //    Hold G1 right trigger → auto-align (lights blink/solid red)
            //    Release while within ±5° → fire motif kick sequence
            //    Release outside ±5° → cancel, restore ball-color lights
            //------------------------------------------------------------------------------------------
            boolean alignTriggered = gamepad1.right_trigger > 0.2;

            if (alignTriggered) {
                _driveUtilsAdvanced.autoAlign();
            } else if (wasAlignTriggered) {
                // Trigger just released — decide whether to fire
                double angle = _driveUtilsAdvanced.getAlignmentAngle();
                if (Math.abs(angle) <= 5.0) {
                    _kickMotif.setKick(isThreeBallMode);
                }
                _driveUtilsAdvanced.endAutoAlign();
            }
            wasAlignTriggered = alignTriggered;

            // G2 left stick click: cancel alignment at any time
            if (gamepad2.left_stick_button) {
                _driveUtilsAdvanced.endAutoAlign();
            }

            _driveUtilsAdvanced.printCalcDiff();
            _driveUtilsAdvanced.printXDegrees();
            _driveUtilsAdvanced.driveMecanum(gamepad1, _robot.kickers);

            //------------------------------------------------------------------------------------------
            //--- Shooter Speed Control
            //------------------------------------------------------------------------------------------
            if (gamepad2.left_bumper && (shooterSpeedRpm > 0)) {
                if (shooterSpeedRpm > 10) {
                    shooterSpeedRpm -= 10;
                    shooterSpeedRpm3Ball -= 10;
                } else {
                    shooterSpeedRpm = 0;
                    shooterSpeedRpm3Ball = 0;
                }
            }
            if (gamepad2.right_bumper && (shooterSpeedRpm < 6000)) {
                if (shooterSpeedRpm < 5990) {
                    shooterSpeedRpm += 10;
                    shooterSpeedRpm3Ball += 10;
                } else {
                    shooterSpeedRpm = 6000;
                    shooterSpeedRpm3Ball = 6000;
                }
            }

            double dist = _driveUtilsAdvanced.getDist();

            if (!isThreeBallMode && isAutoSpeed) {
                shooterSpeedRpm = Math.round((float) (2667 + (dist * -31.6) + (0.597 * (dist * dist)) -
                        (0.00375 * dist * dist * dist) + (0.00000895 * dist * dist * dist * dist)));
            } else {
                shooterSpeedRpm3Ball = Math.round((float) ((dist * dist * 0.0621) - (0.707 * dist) + 2414.16));
            }

            if (gamepad1.xWasPressed()) {
                isAutoSpeed = !isAutoSpeed;
            }
            if (gamepad2.y) {
                robotPosition = position.Close;
                shooterSpeedRpm3Ball = SPEED_3BALL_CLOSE;
                shooterSpeedRpm = SPEED_1BALL_CLOSE;
            }
            if (gamepad2.x) {
                robotPosition = position.Medium;
                shooterSpeedRpm3Ball = SPEED_3BALL_MEDIUM;
                shooterSpeedRpm = SPEED_1BALL_MEDIUM;
            }
            if (gamepad2.a) {
                robotPosition = position.Far;
                shooterSpeedRpm3Ball = SPEED_3BALL_FAR;
                shooterSpeedRpm = SPEED_1BALL_FAR;
            }
            if (gamepad2.b) {
                _robot.intake.stop();
            }
            if (g1RE.RisingEdgeButton(gamepad1, "y")) {
                isThreeBallMode = !isThreeBallMode;
            }

            if (isThreeBallMode) {
                _robot.shooter.shoot(shooterSpeedRpm3Ball);
            } else {
                _robot.shooter.shoot(shooterSpeedRpm);
            }

            //------------------------------------------------------------------------------------------
            //--- Intake & Ball Detection
            //------------------------------------------------------------------------------------------
            _robot.intake.run(overrideBallDistanceDetection);
            if (gamepad1.b) {
                _robot.kickers.kickMiddle();
                overrideBallDistanceDetection = true;
                overrideTimer.reset();
            }
            if (overrideTimer.seconds() > 2 && overrideBallDistanceDetection) {
                overrideBallDistanceDetection = false;
            }

            //------------------------------------------------------------------------------------------
            //--- 3-Ball Detection: rumble both controllers on rising edge
            //------------------------------------------------------------------------------------------
            boolean curr3Balls = _robot.intake.isAll3Detected();
            if (curr3Balls && !prev3Balls) {
                gamepad1.rumble(0.9, 0.9, 500);
                gamepad2.rumble(0.9, 0.9, 500);
            }
            prev3Balls = curr3Balls;

            //------------------------------------------------------------------------------------------
            //--- Alignment Light Override
            //    Runs AFTER intake.run() (which sets ball-color lights) so it takes priority.
            //    Aligned + at speed → solid red; aligning → fast-blink red
            //------------------------------------------------------------------------------------------
            if (_driveUtilsAdvanced.isAligning) {
                double angle = _driveUtilsAdvanced.getAlignmentAngle();
                double targetSpeedTps = isThreeBallMode
                        ? (double) shooterSpeedRpm3Ball * Shooter.ticksPerRotation / 60.0
                        : (double) shooterSpeedRpm * Shooter.ticksPerRotation / 60.0;
                boolean isReadyToShoot = Math.abs(angle) <= 5.0 && shooterSpeed >= targetSpeedTps * 0.95;

                Lights.Blink blinkMode = isReadyToShoot ? Lights.Blink.NONE : Lights.Blink.FAST;
                _robot.lights.setLeft(Lights.Color.RED, blinkMode);
                _robot.lights.setMiddle(Lights.Color.RED, blinkMode);
                _robot.lights.setRight(Lights.Color.RED, blinkMode);
            }

            //------------------------------------------------------------------------------------------
            //--- Telemetry (throttled — reduces WiFi/serial overhead)
            //------------------------------------------------------------------------------------------
            if ((loop_count % 5) == 0) {
                telemetry.addData("target speed one ball in rpm", shooterSpeedRpm);
                telemetry.addData("target speed 3 ball in rpm", shooterSpeedRpm3Ball);
                telemetry.addData("three ball mode", isThreeBallMode);
                telemetry.addData("auto speed mode", isAutoSpeed);
                telemetry.addData("robot shooting position", robotPosition.toString());
                telemetry.addData("shooter ticks/sec", shooterSpeed);
                telemetry.addData("game motif", MotifKicking.GameMotif.toString());
                telemetry.addData("ll stored motif", _robot.limelightHardware2Axis.storedGameMotif != null
                        ? _robot.limelightHardware2Axis.storedGameMotif.toString() : "none");
                telemetry.addData("kickstand deployed", _robot.kickstand.isDeployed());
                _robot.shooter.getTelemetry();
            }

            telemetry.update();
        }
    }
}
