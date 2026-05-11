package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Intake_Incomplete;
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
//  - Dpad Up           - Kickers to middle + override ball distance detect (~2s)
//  - Dpad Down         - Toggle kickstand (0 deg OFF / 90 deg ON)
//  - Dpad Left         - Rotate Left (Slow)
//  - Dpad Right        - Rotate Right (Slow)
//
//  - Right Trigger     - Hold to auto-align; auto-fires when within SHOOT_TOLERANCE_DEG
//  - Right Bumper      - Cancel auto-align
//
//  - Y (▲)             - Toggle 3-ball / 1-ball mode
//  - A (✕)             - Fire: 3-ball simultaneous or 1-ball sequential motif order
//  - X (■)             - Toggle auto-speed on/off
//  - B (○)             - Toggle auto-align assist on/off
//
//----------------------------------------------------------------------
// Joystick 2 -----------------------------------------------------------
//  - Dpad Left         - Manual kick left   (requires preset selected)
//  - Dpad Up           - Manual kick middle (requires preset selected)
//  - Dpad Right        - Manual kick right  (requires preset selected)
//  - Dpad Down         - Manual kick all 3  (requires preset selected)
//
//  - Right Trigger     - Manual intake  (overrides auto)
//  - Left Trigger      - Manual outtake (overrides auto)
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
    public static double SHOOTER_READY_RATIO = 0.98;
    // Angle tolerance for auto-fire: shoot if within this many degrees when trigger is released
    // (or while trigger is held once aligned). Wider than ALIGN_TARGET_DEG on purpose.
    public static double SHOOT_TOLERANCE_DEG = 1.5;
    public static double ODOMETRY_SPEED_BLEND = 0.65;
    public static int ODOMETRY_RPM_MIN = 0;
    public static int ODOMETRY_RPM_MAX = 6000;
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
    Boolean isBlue = false;
    DriveUtilsAdvanced _driveUtilsAdvanced;

    public ElapsedTime _runtime = new ElapsedTime();

    MotifKicking _kickMotif = new MotifKicking(_robot);

    public boolean overrideBallDistanceDetection = false;
    public ElapsedTime overrideTimer = new ElapsedTime();


    enum position {
        Close,
        Medium,
        Far,
        None
    }

    private double targetShooterTps(boolean isThreeBallMode, int oneBallRpm, int threeBallRpm) {
        int targetRpm = isThreeBallMode ? threeBallRpm : oneBallRpm;
        return (double) targetRpm * Shooter.ticksPerRotation / 60.0;
    }

    private boolean readyToShoot(boolean isThreeBallMode, int oneBallRpm, int threeBallRpm, double shooterSpeed) {
        double targetSpeedTps = targetShooterTps(isThreeBallMode, oneBallRpm, threeBallRpm);
        double alignError = Math.abs(_driveUtilsAdvanced.getLastAlignErrorDeg());
        return alignError <= SHOOT_TOLERANCE_DEG
                && targetSpeedTps > 0
                && shooterSpeed >= targetSpeedTps * SHOOTER_READY_RATIO;
    }

    private int odometryOneBallRpm(double dist) {
        int rpm = Math.round((float) (2667 + (dist * -31.6) + (0.597 * (dist * dist)) -
                (0.00375 * dist * dist * dist) + (0.00000895 * dist * dist * dist * dist)));
        return Math.max(ODOMETRY_RPM_MIN, Math.min(ODOMETRY_RPM_MAX, rpm));
    }

    private int odometryThreeBallRpm(double dist) {
        int rpm = Math.round((float) ((dist * dist * 0.0621) - (0.707 * dist) + 2414.16));
        return Math.max(ODOMETRY_RPM_MIN, Math.min(ODOMETRY_RPM_MAX, rpm));
    }

    private int blendPresetWithOdometry(int presetRpm, int odometryRpm) {
        if (presetRpm <= 0) {
            return odometryRpm;
        }
        return Math.round((float) (presetRpm * (1.0 - ODOMETRY_SPEED_BLEND)
                + odometryRpm * ODOMETRY_SPEED_BLEND));
    }

    private Lights.Color motifCharToLight(char motifColor) {
        return motifColor == 'G' ? Lights.Color.GREEN : Lights.Color.PURPLE;
    }

    private void showMotifLights(LimelightHardware2Axis.Motif motif) {
        if (motif == null) {
            _robot.lights.setLeft(Lights.Color.WHITE, Lights.Blink.FAST);
            _robot.lights.setMiddle(Lights.Color.WHITE, Lights.Blink.FAST);
            _robot.lights.setRight(Lights.Color.WHITE, Lights.Blink.FAST);
            return;
        }
        char[] pattern = motif.toString().toCharArray();
        _robot.lights.setLeft(motifCharToLight(pattern[0]));
        _robot.lights.setMiddle(motifCharToLight(pattern[1]));
        _robot.lights.setRight(motifCharToLight(pattern[2]));
    }

    private void showNoGoalTagSearchLights() {
        int phase = ((int) (_runtime.seconds() / 0.2)) % 3;
        Lights.Color color = phase == 0
                ? Lights.Color.RED
                : (phase == 1 ? Lights.Color.ORANGE : Lights.Color.YELLOW);
        _robot.lights.setLeft(color);
        _robot.lights.setMiddle(color);
        _robot.lights.setRight(color);
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
        boolean autoAlignEnabled = true;
        int presetOneBallRpm = 0;
        int presetThreeBallRpm = 0;

        position robotPosition = position.None;

        // Trigger-release auto-align state
        boolean wasAlignTriggered = false;
        // Armed on each fresh trigger press; disarmed once the shot fires so we don't re-fire every loop.
        boolean autoFireArmed = false;
        // Set when a blip is triggered mid-alignment to reposition balls for re-read. Cleared on each
        // new trigger press. If still INVALID after one blip, falls back to sequential.
        boolean autoFireBlipped = false;
        // Rising-edge tracker for 3-ball auto-blip
        boolean wasAll3 = false;

        // Kick-completion detection: restart intake when sequence finishes
        boolean wasKicking = false;

        _robot.init(robotVersion);
        telemetry.addData("location string in teleopState", Location.GetPose());
        telemetry.addData("Class Hash in teleopState", Location.class.hashCode());
        _driveUtilsAdvanced = new DriveUtilsAdvanced(hardwareMap, Location.pose, _robot.drive,
                _robot.limelightHardware2Axis, this.telemetry, isBlue, _robot);

        RisingEdge g1RE = new RisingEdge();
        RisingEdge g2RE = new RisingEdge();

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

            //------------------------------------------------------------------------------------------
            //--- Lights blink state machine
            //------------------------------------------------------------------------------------------
            _robot.lights.run();

            // Lock intake's updateLights() when aligning so red override wins.
            // Must be set before intake.run() below.
            _robot.intake.lightsLocked = _driveUtilsAdvanced.isAligning;

            //------------------------------------------------------------------------------------------
            //--- Kickers: G2 dpad manual override always active via runFinal()
            //------------------------------------------------------------------------------------------
            // Manual D-pad firing should always be preset-based (not auto-speed/odometry).
            // If no preset is selected, target remains 0 and runFinal() will not fire.
            boolean g2ManualKickCommand =
                    gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_down;
            int manualOneBallRpm = presetOneBallRpm;
            int manualThreeBallRpm = presetThreeBallRpm;

            // While driver/operator is manually commanding a kick, command shooter to the preset
            // so the speed gate in runFinal() matches what the shooter is actually targeting.
            if (g2ManualKickCommand && manualOneBallRpm > 0 && manualThreeBallRpm > 0) {
                shooterSpeedRpm = manualOneBallRpm;
                shooterSpeedRpm3Ball = manualThreeBallRpm;
            }

            if (_robot.kickers.runFinal((double) manualOneBallRpm * Shooter.ticksPerRotation / 60,
                    shooterSpeed, true,
                    (double) manualThreeBallRpm * Shooter.ticksPerRotation / 60,
                    -1, _robot.intake)) {
                _driveUtilsAdvanced.endAutoAlign();
            }

            //------------------------------------------------------------------------------------------
            //--- G1 A: fire kick sequence (non-blocking)
            //    3-ball mode = simultaneous, 1-ball mode = sequential motif order
            //------------------------------------------------------------------------------------------
            if (g1RE.RisingEdgeButton(gamepad1, "a") && !_kickMotif.isKicking()) {
                if (!isThreeBallMode && (!MotifKicking.isFieldMotifKnown(_robot)
                        || !MotifKicking.currentBallsMatchFieldMotif(_robot))) {
                    _kickMotif.setKickLeftToRightSequential();
                } else {
                    _kickMotif.setKick(isThreeBallMode);
                }
            }
            _kickMotif.checkKick();

            // Rising/falling-edge: kick sequence start and end
            boolean isKickingNow = _kickMotif.isKicking();
            if (wasKicking && !isKickingNow) {
                // Kick sequence finished — stop the long rumble and clear ball detection
                gamepad1.stopRumble();
                _robot.intake.clearAllSensorValues();
            }
            wasKicking = isKickingNow;

            //------------------------------------------------------------------------------------------
            //--- Kickstand: G1 dpad down toggles 0 deg / 90 deg
            //------------------------------------------------------------------------------------------
            _robot.kickstand.run();

            //------------------------------------------------------------------------------------------
            //--- Camera & Localizer
            //------------------------------------------------------------------------------------------
            _driveUtilsAdvanced.updateCameraPitch();
            _robot.limelightHardware2Axis.loop(_driveUtilsAdvanced.getHeadingDegrees());
            _robot.limelightHardware2Axis.servos();
            LimelightHardware2Axis.Motif savedMotif =
                    _robot.limelightHardware2Axis.updateStoredGameMotif(!isThreeBallMode);
            if (savedMotif != null) {
                MotifKicking.updateMotif(savedMotif);
            }
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
                if (!wasAlignTriggered) {
                    // First loop of trigger press only: arm auto-fire and enable auto-speed.
                    // Doing this once prevents the trigger from overriding G2 X (auto-speed toggle)
                    // every loop while held.
                    autoFireArmed = true;
                    autoFireBlipped = false;
                    isAutoSpeed = true;
                    // Start a continuous rumble on G1 — held through alignment and kicker firing.
                    // Stopped by stopRumble() when the kick sequence completes or trigger is released without firing.
                    gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                }
                _driveUtilsAdvanced.autoAlign();
                // Auto-fire: once aligned within SHOOT_TOLERANCE_DEG and shooter is ready, fire once.
                if (autoFireArmed && !_kickMotif.isKicking()
                        && readyToShoot(isThreeBallMode, shooterSpeedRpm, shooterSpeedRpm3Ball, shooterSpeed)) {
                    if (!isThreeBallMode && (!MotifKicking.isFieldMotifKnown(_robot)
                            || !MotifKicking.currentBallsMatchFieldMotif(_robot))) {
                        // Motif unknown or wrong balls — fire sequential
                        _kickMotif.setKickLeftToRightSequential();
                        autoFireArmed = false;
                    } else if (!isThreeBallMode
                            && MotifKicking.intakeMotifFromRobot(_robot) == MotifKicking.Motif.INVALID
                            && !autoFireBlipped) {
                        // Motif known, balls match multiset, but color sensors can't read arrangement —
                        // blip once to reposition balls for re-read. Keep autoFireArmed to retry.
                        _robot.kickers.kickMiddle();
                        overrideBallDistanceDetection = true;
                        overrideTimer.reset();
                        autoFireBlipped = true;
                    } else {
                        // Arrangement readable (or already blipped once) — fire in motif order
                        _kickMotif.setKick(isThreeBallMode);
                        autoFireArmed = false;
                    }
                }
            } else if (wasAlignTriggered) {
                // Trigger just released — fire if still within tolerance (covers slow alignments)
                if (autoFireArmed
                        && readyToShoot(isThreeBallMode, shooterSpeedRpm, shooterSpeedRpm3Ball, shooterSpeed)) {
                    if (!isThreeBallMode && (!MotifKicking.isFieldMotifKnown(_robot)
                            || !MotifKicking.currentBallsMatchFieldMotif(_robot))) {
                        _kickMotif.setKickLeftToRightSequential();
                    } else {
                        // On release, fire whatever arrangement is readable; INVALID falls back to GPP default
                        _kickMotif.setKick(isThreeBallMode);
                    }
                }
                // Trigger released without a kick firing — stop the continuous rumble
                gamepad1.stopRumble();
                _driveUtilsAdvanced.endAutoAlign();
                autoFireArmed = false;
                autoFireBlipped = false;
            }
            wasAlignTriggered = alignTriggered;

            // Manual auto-align gating: end alignment if driver has disabled it and trigger isn't held
            if (!alignTriggered && !autoAlignEnabled && _driveUtilsAdvanced.isAligning) {
                _driveUtilsAdvanced.endAutoAlign();
            }


            // G2 left stick click: toggle auto-align assist on/off (does NOT block trigger auto-shoot)
            if (g2RE.RisingEdgeButton(gamepad2, "left_stick_button")) {
                autoAlignEnabled = !autoAlignEnabled;
                if (!autoAlignEnabled) {
                    _driveUtilsAdvanced.endAutoAlign();
                }
            }

            // G2 right stick click: toggle auto-speed on/off
            if (g2RE.RisingEdgeButton(gamepad2, "right_stick_button")) {
                isAutoSpeed = !isAutoSpeed;
            }

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

            // Spin up shooter automatically when starting auto-align (no confirmation / no fail state).
            if (alignTriggered && !wasAlignTriggered) {
                int odo1 = odometryOneBallRpm(dist);
                int odo3 = odometryThreeBallRpm(dist);
                shooterSpeedRpm = blendPresetWithOdometry(presetOneBallRpm, odo1);
                shooterSpeedRpm3Ball = blendPresetWithOdometry(presetThreeBallRpm, odo3);
            }

            if (isAutoSpeed) {
                shooterSpeedRpm = blendPresetWithOdometry(presetOneBallRpm, odometryOneBallRpm(dist));
                shooterSpeedRpm3Ball = blendPresetWithOdometry(presetThreeBallRpm, odometryThreeBallRpm(dist));
            }

            if (g2RE.RisingEdgeButton(gamepad2, "y")) {
                robotPosition = position.Close;
                presetThreeBallRpm = SPEED_3BALL_CLOSE;
                presetOneBallRpm = SPEED_1BALL_CLOSE;
                shooterSpeedRpm3Ball = presetThreeBallRpm;
                shooterSpeedRpm = presetOneBallRpm;
                if (!gamepad2.isRumbling()) gamepad2.rumbleBlips(1); // 1 blip = Close
            }
            if (g2RE.RisingEdgeButton(gamepad2, "x")) {
                robotPosition = position.Medium;
                presetThreeBallRpm = SPEED_3BALL_MEDIUM;
                presetOneBallRpm = SPEED_1BALL_MEDIUM;
                shooterSpeedRpm3Ball = presetThreeBallRpm;
                shooterSpeedRpm = presetOneBallRpm;
                if (!gamepad2.isRumbling()) gamepad2.rumbleBlips(2); // 2 blips = Medium
            }
            if (g2RE.RisingEdgeButton(gamepad2, "a")) {
                robotPosition = position.Far;
                presetThreeBallRpm = SPEED_3BALL_FAR;
                presetOneBallRpm = SPEED_1BALL_FAR;
                shooterSpeedRpm3Ball = presetThreeBallRpm;
                shooterSpeedRpm = presetOneBallRpm;
                if (!gamepad2.isRumbling()) gamepad2.rumbleBlips(3); // 3 blips = Far
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

            // Auto-blip: when all 3 balls are first detected, automatically kick middle
            // and override distance detection (same as G1 dpad up).
            boolean all3 = _robot.intake._leftBallColor != Intake_Incomplete.BallColor.NONE
                    && _robot.intake._leftBallColor != Intake_Incomplete.BallColor.UNKNOWN
                    && _robot.intake._middleBallColor != Intake_Incomplete.BallColor.NONE
                    && _robot.intake._middleBallColor != Intake_Incomplete.BallColor.UNKNOWN
                    && _robot.intake._rightBallColor != Intake_Incomplete.BallColor.NONE
                    && _robot.intake._rightBallColor != Intake_Incomplete.BallColor.UNKNOWN;
            // Guard: !overrideBallDistanceDetection prevents re-triggering after the sensor averages
            // are cleared below — colors go to NONE briefly, wasAll3 resets, and without this guard
            // the blip would fire again as soon as the sensors re-detect all 3 balls.
            if (all3 && !wasAll3 && !overrideBallDistanceDetection) {
                _robot.kickers.kickMiddle();
                overrideBallDistanceDetection = true;
                overrideTimer.reset();
                // Reset rolling averages so sensor reads after the blip reflect new ball positions.
                // Lights will update within a few loops as fresh readings rebuild the averages.
                _robot.intake.clearAllSensorValues();
            }
            wasAll3 = all3;

            if (g1RE.RisingEdgeButton(gamepad1, "right_bumper")) {
                _robot.kickers.kickMiddle();
                overrideBallDistanceDetection = true;
                overrideTimer.reset();
                // Reset rolling averages so sensor reads after the blip reflect new ball positions.
                _robot.intake.clearAllSensorValues();
            }
            if (overrideTimer.seconds() > 2 && overrideBallDistanceDetection) {
                overrideBallDistanceDetection = false;
            }

            //------------------------------------------------------------------------------------------
            //--- Alignment Light Override
            //    Runs AFTER intake.run() (which sets ball-color lights) so it takes priority.
            //    Aligned + at speed → solid red; aligning → fast-blink red
            //------------------------------------------------------------------------------------------
            if (_driveUtilsAdvanced.isAligning) {
                boolean isReadyToShoot = readyToShoot(isThreeBallMode, shooterSpeedRpm, shooterSpeedRpm3Ball, shooterSpeed);

                if (!_driveUtilsAdvanced.hasGoalTag()) {
                    showNoGoalTagSearchLights();
                } else if (isReadyToShoot) {
                    _robot.lights.setLeft(Lights.Color.RED);
                    _robot.lights.setMiddle(Lights.Color.RED);
                    _robot.lights.setRight(Lights.Color.RED);
                } else {
                    // Alternate red blink with a status color:
                    //   orange → fewer than 3 balls loaded
                    //   white  → 3 balls loaded but colors don't match the field motif
                    //   yellow → motif unknown (don't know which ball to fire)
                    //   red    → all other cases (aligning — driver is committed to shooting)
                    boolean phaseRed = (((int) (_runtime.seconds() / 0.25)) % 2 == 0);
                    Lights.Color nonRedColor;
                    if (!_robot.intake.isAll3Detected()) {
                        nonRedColor = Lights.Color.ORANGE;
                    } else if (MotifKicking.isFieldMotifKnown(_robot) && !MotifKicking.currentBallsMatchFieldMotif(_robot)) {
                        nonRedColor = Lights.Color.WHITE;
                    } else if (!isThreeBallMode && !MotifKicking.isFieldMotifKnown(_robot)) {
                        nonRedColor = Lights.Color.YELLOW;
                    } else {
                        nonRedColor = Lights.Color.RED;
                    }
                    Lights.Color blinkColor = phaseRed ? Lights.Color.RED : nonRedColor;
                    _robot.lights.setLeft(blinkColor, Lights.Blink.FAST);
                    _robot.lights.setMiddle(blinkColor, Lights.Blink.FAST);
                    _robot.lights.setRight(blinkColor, Lights.Blink.FAST);
                }
            }

            //------------------------------------------------------------------------------------------
            //--- Telemetry (throttled — reduces WiFi/serial overhead)
            //------------------------------------------------------------------------------------------
            if ((loop_count % 5) == 0) {
                telemetry.addData("Run Time", " " + _runtime.toString() + " Loop Count:" + loop_count);
                telemetry.addData("ball mode", isThreeBallMode ? "3 BALL" : "1 BALL");
                telemetry.addData("AUTO SPEED", isAutoSpeed ? "ON" : "OFF");
                telemetry.addData("AUTO ALIGN TOGGLE", autoAlignEnabled ? "ON" : "OFF");
                telemetry.addData("AUTO SHOOT (G1 RT)", alignTriggered ? "ACTIVE (forces align+auto speed)" : "idle");
                telemetry.addData("preset", robotPosition.toString());
                telemetry.addData("rpm source",
                        g2ManualKickCommand
                                ? "PRESET (manual kick)"
                                : (isAutoSpeed ? "ODOMETRY (blended)" : "MANUAL/PRESET"));
                telemetry.addData("target speed one ball in rpm", shooterSpeedRpm);
                telemetry.addData("target speed 3 ball in rpm", shooterSpeedRpm3Ball);
                telemetry.addData("robot shooting position", robotPosition.toString());
                telemetry.addData("shooter ticks/sec", shooterSpeed);
                telemetry.addData("goal tag", _driveUtilsAdvanced.hasGoalTag() ? "locked" : "odometry fallback");
                telemetry.addData("align error deg", "%.2f", _driveUtilsAdvanced.getLastAlignErrorDeg());
                telemetry.addData("align power", "%.3f", _driveUtilsAdvanced.getLastAlignPower());
                telemetry.addData("align stable loops", _driveUtilsAdvanced.getAlignStableLoops());
                telemetry.addData("ready to shoot", readyToShoot(isThreeBallMode, shooterSpeedRpm, shooterSpeedRpm3Ball, shooterSpeed));
                telemetry.addData("game motif", MotifKicking.GameMotif.toString());
                telemetry.addData("ll stored motif", _robot.limelightHardware2Axis.storedGameMotif != null
                        ? _robot.limelightHardware2Axis.storedGameMotif.toString() : "none");
                telemetry.addData("kickstand deployed", _robot.kickstand.isDeployed());
                _robot.shooter.getTelemetry();
                telemetry.update();
            }
        }
    }
}
