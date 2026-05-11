package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Kickers;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

//TODO: important things to do
//TODO: make it only auto shoot the balls after align if in the shooting zone
//TODO: fix the camera
//TODO: try to use the dcmotorex on shooters to detect current
//TODO: only reset position if auto was the last thing run
//auto reverse intake
//lights flash red when shoot
//shooting while moving
//turn faster
@Config
public class DriveUtilsAdvanced {
    // --- Alignment tuning (edit here OR live-tune via FTC Dashboard) ---
    public static double ALIGN_KP = 0.7;         // proportional gain: deg → turn power
    public static double ALIGN_KD = 0.003;       // derivative gain: damps oscillations
    public static double ALIGN_MIN_POWER = 0.07; // minimum power to overcome static friction
    public static double ALIGN_FINE_DEG = 8;   // within this angle → stop turning (done)
    // -------------------------------------------------------------------

    // PD state — reset each time alignment starts/ends
    public static double ALIGN_PID_KP = 0.035;       // degrees -> turn power
    public static double ALIGN_PID_KI = 0.0;         // keep 0 unless the robot consistently stops off-center
    public static double ALIGN_PID_KD = 0.0025;      // damps fast approach near target
    public static double ALIGN_PID_KS = 0.045;       // static-friction feedforward
    public static double ALIGN_PID_KS_END_DEG = 1.0; // no static push inside this range, prevents close wiggle
    public static double ALIGN_PID_MAX_POWER = 0.65;
    public static double ALIGN_PID_MAX_ACCEL = 0.12;         // max turn-power change per loop
    public static double ALIGN_FALLBACK_MAX_POWER = 0.15;    // speed cap while searching for goal tag; lower = less overshoot on acquisition
    public static double ALIGN_TARGET_DEG = 1.5;
    public static double ALIGN_INTEGRAL_LIMIT = 120.0;
    public static int ALIGN_STABLE_LOOPS = 4;
    public static double ALIGN_MANUAL_YAW_OVERRIDE = 0.08;
    public static boolean SHOW_DASHBOARD_DEBUG = false;

    // --- TeleOp strafe heading-hold assist (does NOT run during auto-align) ---
    public static boolean STRAFE_HOLD_ENABLED = false;
    public static double STRAFE_HOLD_KP = 2.2;               // rad error -> turn power
    public static double STRAFE_HOLD_MAX_POWER = 0.35;
    public static double STRAFE_HOLD_STRAFE_DEADBAND = 0.25; // left stick X
    public static double STRAFE_HOLD_AXIAL_MAX = 0.20;       // left stick Y
    public static double STRAFE_HOLD_YAW_DEADBAND = 0.12;    // right stick X

    // Camera yaw search was removed — panning the camera horizontally while searching created a
    // feedback loop: the pan moved the goal tag out of FOV, hasGoalTag() returned false, and the
    // robot panned more. The fix is to keep the camera fixed (yaw=0) and use MegaTag bearing
    // estimation instead (see autoAlignViaLLandPower fallback below).

    private boolean _strafeHoldActive = false;
    private double _strafeHoldHeadingRad = 0.0;

    private double _alignPrevError = 0;
    private double _alignIntegral = 0;
    private double _lastAlignPower = 0;
    private double _lastAlignErrorDeg = 180.0;
    private int _alignStableLoops = 0;
    private boolean _alignFirstLoop = true; // seed prevError on first loop to avoid spike
    private final ElapsedTime _alignDtTimer = new ElapsedTime();

    private List<Action> runningActions = new ArrayList<>();
    // Pre-allocated to avoid creating a new ArrayList every loop iteration.
    private final List<Action> _newActions = new ArrayList<>();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private boolean isBlue = false;
    public boolean isAligning = false;
    // Throttle counter — dashboard/telemetry spam is sent every N loops only.
    private int _telemetryThrottle = 0;
    private static final int TELEMETRY_EVERY_N_LOOPS = 5;
    private double x = 0;

    private double x2 = 60;
    private double x3 = 66;
    private double x4 = 72;
    private double dxdt = 0;
    private double y = 0;
    private double y2 = 60;
    private double y3 = 60;

    private double y4 = 72;
    private double dydt = 0;
    private double heading = 0;
    private double yaw = 0;

    private double tagXposition = 0.0;
    private int targetTagId = 0;

    private RobotHardware _robot;

    private Telemetry telemetry;

    public Drive drive;

    private MecanumDrive driveClass;

    private LimelightHardware2Axis limelightHardware2Axis;

    double adjustmentDegrees(){//higher value will turn it more clockwise
        if(isBlue){
            return -2; // 2° counter-clockwise offset for blue alliance goal tag
        }else{
            return 0; // no offset for red alliance
        }
     };
    public DriveUtilsAdvanced(HardwareMap hardwareMap, Pose2d pose, Drive drive,LimelightHardware2Axis limelightHardware2Axis,
                              Telemetry telemetry,Boolean isBlue, RobotHardware robotHardware){
        driveClass = new MecanumDrive(hardwareMap, pose);
        isAligning=false;
        this.drive = drive;
        this.isBlue=isBlue;
        this.telemetry = telemetry;
        this.limelightHardware2Axis = limelightHardware2Axis;
        this._robot = robotHardware;
        if (isBlue)
        {
            this.targetTagId = 20;
        }
        else
        {
            this.targetTagId = 24;
        }
        telemetry.addData("location string in driveUtils", Location.GetPose());
        telemetry.addData("Class Hash in driveUtils", Location.class.hashCode());
        telemetry.addData("rr pinpoint localizer", driveClass.localizer.getPose().toString());

    }


    private void setVars(double x, double dxdt, double y, double dydt, double heading, double yaw){
        this.x = x;
        this.x2 = 60+x;//60 bc centered around the april tag not corner
        this.x3 = 60+x+cos(heading)*11;
        this.x4 = 72+x;
        this.dxdt = dxdt;
        this.y = y;
        if(isBlue){
            this.y2 = 60+y;
            this.y3 = 60+y+sin(heading)*11;
            this.y4 = 72+y;
        }else{
            this.y2 = 60-y;
            this.y3 = 60-y-sin(heading)*11;
            this.y4 = 72-y;
        }

        this.dydt = dydt;
        this.heading = heading;
        this.yaw = yaw;// yaw is posotive goes counterclockwise
        // Only push debug telemetry every N loops to reduce hub communication overhead.
        if (SHOW_DASHBOARD_DEBUG && _telemetryThrottle % TELEMETRY_EVERY_N_LOOPS == 0) {
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("x speed", dxdt);
            telemetry.addData("y speed", dydt);
            telemetry.addData("heading", heading);
            telemetry.addData("heading speed", yaw);
            telemetry.addData("distance", Math.sqrt(x2*x2+y2*y2));
            double _tgtHeading = getTargetHeading(y4-14.55098425, x4-11.82122047);
            telemetry.addData("target heading degrees", Math.toDegrees(_tgtHeading));
            telemetry.addData("calcDiff degrees", Math.toDegrees(calcDifference(_tgtHeading)));
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", x);
            packet.put("y", y);
            packet.put("x speed", dxdt);
            packet.put("y speed", dydt);
            packet.put("heading", heading);
            packet.put("heading speed", yaw);
            packet.put("x2", x2);
            packet.put("y2", y2);
            packet.put("distance", Math.sqrt(x2*x2+y2*y2));
            packet.put("target heading degrees", Math.toDegrees(_tgtHeading));
            packet.put("calcDiff degrees", Math.toDegrees(calcDifference(_tgtHeading)));
            dashboard.sendTelemetryPacket(packet);
        }
    }
    private void setVarsAdvanced(double x, double y, double heading,double axial, double lateral, double yaw){
        setVars(x,lateral*sin(heading)+axial*cos(heading),y,axial*sin(heading)-lateral*cos(heading),heading,yaw);

    };

    // returns true if trying to auto align and the auto align is finished
    public boolean driveMecanum(Gamepad gamepad, Kickers kickers){
        boolean returnn = false;
        _telemetryThrottle++;

        driveClass.localizer.update();
        // Cache pose once to avoid repeated getPose() calls below.
        com.acmerobotics.roadrunner.Pose2d currentPose = driveClass.localizer.getPose();
        setVarsAdvanced(
                currentPose.position.x,
                currentPose.position.y,
                currentPose.heading.toDouble(),
                -gamepad.left_stick_y * drive.getSpeedMultiplier(),
                gamepad.left_stick_x * drive.getSpeedMultiplier(),
                -gamepad.right_stick_x * drive.getSpeedMultiplierRotate()
        );
        //driveClass.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad.left_stick_x, gamepad.left_stick_y), gamepad.right_stick_x));
        //drive.driveControl(1);

        // Throttle dashboard robot-drawing to every N loops (no need to redraw every 10ms).
        TelemetryPacket packet2 = new TelemetryPacket();
        if (_telemetryThrottle % TELEMETRY_EVERY_N_LOOPS == 0) {
            if (SHOW_DASHBOARD_DEBUG) {
                TelemetryPacket packet = new TelemetryPacket();
                Canvas c = packet.fieldOverlay();
                Drawing.drawRobot(c, currentPose);
                packet.put("change in heading", thetadt());
                dashboard.sendTelemetryPacket(packet);
            }
        }

        // update running actions — use Iterator to remove finished actions in-place,
        // avoiding a new ArrayList allocation every loop.
        boolean skipDrive = false;
        if(runningActions.isEmpty()){
            if (SHOW_DASHBOARD_DEBUG) {
                packet2.addLine("RR is not running");
            }
        }
        else
        {
            java.util.Iterator<Action> iter = runningActions.iterator();
            while (iter.hasNext()) {
                Action action = iter.next();
                if (SHOW_DASHBOARD_DEBUG) {
                    packet2.addLine("RR is running");
                    action.preview(packet2.fieldOverlay());
                }
                if (action.run(packet2)) {
                    skipDrive = true;  // action still running — keep it
                } else {
                    iter.remove();    // action finished — remove it
                }
            }
        }

        if (SHOW_DASHBOARD_DEBUG) {
            dashboard.sendTelemetryPacket(packet2);
        }

        double targetHeading = getTargetHeading(y4-14.55098425, x4-11.82122047);
        double calcDif = calcDifference(targetHeading);//calc diff uses road


        // adjustmentDegrees() is applied only inside autoAlignViaLLandPower() — not here.




////if raodrunners drive class is not running then we will run our code and if we are within the correct range of the target heading
//// we will power our motors with speed that is currently proportionaly with the heading angle we have to change*//
//        //we will calculate the heading angle we have to change at first by using all data collected by deadwheels in our localizer
//        //and then we will use only heading data in combination with camera data to calculate the difference
//        // skipDrive=true: robot still under RR control, got to wait
        if(!skipDrive){
            // Heading hold while strafing (TeleOp usability assist).
            // Only applies when NOT auto-aligning and driver is not commanding rotation.
            double yawImportant = 0.0;
            if (STRAFE_HOLD_ENABLED && !isAligning) {
                double lateralCmd = gamepad.left_stick_x;
                double axialCmd = -gamepad.left_stick_y;
                double yawCmd = gamepad.right_stick_x;

                boolean wantsStrafe = Math.abs(lateralCmd) >= STRAFE_HOLD_STRAFE_DEADBAND
                        && Math.abs(axialCmd) <= STRAFE_HOLD_AXIAL_MAX
                        && Math.abs(yawCmd) <= STRAFE_HOLD_YAW_DEADBAND;

                if (wantsStrafe) {
                    if (!_strafeHoldActive) {
                        _strafeHoldActive = true;
                        _strafeHoldHeadingRad = heading;
                    }
                    double err = _strafeHoldHeadingRad - heading;
                    while (err > Math.PI) err -= 2.0 * Math.PI;
                    while (err < -Math.PI) err += 2.0 * Math.PI;
                    yawImportant = clamp(err * STRAFE_HOLD_KP, -STRAFE_HOLD_MAX_POWER, STRAFE_HOLD_MAX_POWER);
                } else {
                    _strafeHoldActive = false;
                }
            } else {
                _strafeHoldActive = false;
            }

            if (isAligning) {
                // When auto-aligning, always use camera-based alignment regardless of odometry
                // heading estimate. Odometry can drift, causing calcDif to exceed ±90° even when
                // the camera can clearly see the goal tag — which would otherwise block alignment
                // and leave the robot stationary.
                returnn = autoAlignViaLLandPower(gamepad, calcDif);
            } else if (calcDif > Math.PI / 2 || calcDif < -Math.PI / 2) {
                // Robot is facing away from goal (odometry) and not aligning — pass driver input through.
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, yawImportant);
            } else {
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, yawImportant);
            }
        }

        return returnn;


    }
//
    private boolean autoAlignViaLLandPower(Gamepad gamepad,double calcDif)
    {
//        if(Math.abs(calcDif)>Math.PI/8) {//this is
//            drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));
//            // todo: fix calcdif auto align first then comment out calcdiff auto align for the smaller angles and use camera
//            //auto aligns using roadrunner position do first
//        }
//        else
//        {
            //double[] distBreakdown = limelightHardware2Axis.getDistanceBreakdown();
            //distBreakdown = null; // temp todo: can we just use
            double angleToTurnFromCamera = limelightHardware2Axis.getTxDegreesForId(this.targetTagId);
            calcDif += Math.toRadians(adjustmentDegrees());
            angleToTurnFromCamera += adjustmentDegrees();
            // Store AFTER applying the adjustment so readyToShoot() compares against the same
            // reference frame the alignment logic uses. Storing pre-adjustment caused readyToShoot()
            // to see e.g. 2.5° while the alignment logic saw 0.5° (adjusted), blocking auto-fire
            // even when the robot was physically aligned.
            _lastAlignErrorDeg = angleToTurnFromCamera;
            if(Math.abs(angleToTurnFromCamera) < ALIGN_TARGET_DEG){ // within fine-alignment threshold
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x,0);
                _lastAlignPower = 0;
                _alignStableLoops++;
                if(_alignStableLoops >= ALIGN_STABLE_LOOPS) {
                    return true;
                }
                else{
                    return false;
                }
            }
            if(angleToTurnFromCamera > (double)120.0){
                _alignStableLoops = 0;
                // Note: _lastAlignPower is intentionally NOT reset here — preserving it lets
                // the accel ramp carry over both from PID→fallback and within the fallback,
                // preventing jerks and ensuring the robot decelerates naturally as it acquires the tag.
                _lastAlignErrorDeg = 180.0;
                _alignIntegral = 0;
                _alignFirstLoop = true;
                // Goal tag not visible. Rotate toward goal using the best available source:
                //
                //  Tier 1 — MegaTag position + Pinpoint heading:
                //            Any visible field tag (obelisk 21/22/23, etc.) gives a full
                //            field-frame robot position via MegaTag. We compute the exact bearing
                //            from that position to the goal (no drift — absolute field coords),
                //            then compare against the Pinpoint IMU heading (more stable than the
                //            MegaTag visual heading estimate). Best of both sensors.
                //
                //  Tier 2 — Pinpoint heading + dead-wheel position (calcDif):
                //            No tags visible at all. Uses Pinpoint IMU heading (no drift) and
                //            dead-wheel field position (slight drift over match). Already
                //            incorporated via calcDif = heading - getTargetHeading(x4, y4).
                double fallbackTarget;
                if (Math.abs(gamepad.right_stick_x) > ALIGN_MANUAL_YAW_OVERRIDE) {
                    fallbackTarget = 0; // driver manually overriding rotation — don't fight them
                } else {
                    Pose2D cameraPos = limelightHardware2Axis.getRobotPos(null);
                    if (cameraPos != null) {
                        // Tier 1: MegaTag position + Pinpoint heading.
                        // getRobotPos() returns FTC field coordinates. Convert to the x4/y4 system
                        // (the same offset+flip that setVars applies to the Road Runner pose) so the
                        // goal constants (11.82, 14.55) are in the same coordinate space.
                        double camX = cameraPos.getX(DistanceUnit.INCH);
                        double camY = cameraPos.getY(DistanceUnit.INCH);
                        double camX4 = 72.0 + camX;
                        double camY4 = isBlue ? 72.0 + camY : 72.0 - camY;
                        double tgtHeading = getTargetHeading(camY4 - 14.55098425, camX4 - 11.82122047);
                        double megaTagErr = heading - tgtHeading; // heading = Pinpoint IMU heading
                        while (megaTagErr >  Math.PI) megaTagErr -= 2 * Math.PI;
                        while (megaTagErr < -Math.PI) megaTagErr += 2 * Math.PI;
                        fallbackTarget = clamp(megaTagErr * 0.4, -ALIGN_FALLBACK_MAX_POWER, ALIGN_FALLBACK_MAX_POWER);
                    } else {
                        // Tier 2: no tags visible — Pinpoint heading + dead-wheel position estimate.
                        fallbackTarget = clamp(calcDif * 0.4, -ALIGN_FALLBACK_MAX_POWER, ALIGN_FALLBACK_MAX_POWER);
                    }
                }
                // Apply the same accel ramp used by the PID so the robot ramps up gradually
                // and — critically — so _lastAlignPower reflects a ramped value when the goal tag
                // first appears and the PID takes over. This prevents the PID from inheriting a
                // sudden speed jump that would cause overshoot.
                double fallbackChange = clamp(fallbackTarget - _lastAlignPower, -ALIGN_PID_MAX_ACCEL, ALIGN_PID_MAX_ACCEL);
                double fallbackTurn = _lastAlignPower + fallbackChange;
                _lastAlignPower = fallbackTurn;
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, fallbackTurn);
            }
            else
            {
                _alignStableLoops = 0;
                double error = angleToTurnFromCamera;
                double derivative = 0;
                double dt = _alignDtTimer.seconds();
                if (_alignFirstLoop) {
                    _alignPrevError = error;
                    _alignFirstLoop = false;
                } else if (dt > 0.001) {
                    derivative = (error - _alignPrevError) / dt;
                }
                if (dt > 0.001 && Math.abs(error) < 10.0) {
                    _alignIntegral += error * dt;
                    _alignIntegral = clamp(_alignIntegral, -ALIGN_INTEGRAL_LIMIT, ALIGN_INTEGRAL_LIMIT);
                } else if (Math.signum(error) != Math.signum(_alignPrevError)) {
                    _alignIntegral = 0;
                }
                _alignPrevError = error;
                _alignDtTimer.reset();

                double staticFeedForward = Math.abs(error) > ALIGN_PID_KS_END_DEG
                        ? Math.signum(error) * ALIGN_PID_KS
                        : 0;

                double speed = (error * ALIGN_PID_KP)
                        + (_alignIntegral * ALIGN_PID_KI)
                        + (derivative * ALIGN_PID_KD)
                        + staticFeedForward;
                speed = clamp(speed, -ALIGN_PID_MAX_POWER, ALIGN_PID_MAX_POWER);
                double change = clamp(speed - _lastAlignPower, -ALIGN_PID_MAX_ACCEL, ALIGN_PID_MAX_ACCEL);
                speed = _lastAlignPower + change;
                _lastAlignPower = speed;
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x,
                        speed);
                //drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));//it is only turning right and not left maybe
                // todo: temp uncomment line above and enable camera
                //auto aligns using roadrunner position do first disable once camera works

            }

//        }

        return false;
    }

    // call from OpMode loop, to see what calcDiff is currently
    public void printCalcDiff(){
        double targetHeading = getTargetHeading(y4-14.55098425, x4-11.82122047);
        double calcDif = calcDifference(targetHeading);

        telemetry.addData("Calc Turn Val:",Math.toDegrees(calcDif));
    }

    public void printXDegrees(){
        double angleToTurnFromCamera = limelightHardware2Axis.getTxDegreesForId(this.targetTagId);
        telemetry.addData("Target X Degrees:", angleToTurnFromCamera);
    }

    //  Use road runner first to get it to be able to see april tag
   public void autoAlign(){
        if (!isAligning) {
            _alignFirstLoop = true;
            _alignIntegral = 0;
            _lastAlignPower = 0;
            _alignStableLoops = 0;
            _alignDtTimer.reset();
        }
        isAligning=true;
    }

    private double getTargetHeading(double targetLocationY, double targetLocationX) {
        // Direction from robot to goal: negate both components of the (robot - goal) vector.
        // Both alliances use the same formula — the previous red branch had the y sign wrong,
        // which caused calcDif to read ~101° off and the odometry fallback to spin backwards.
        return Math.atan2(-targetLocationY, -targetLocationX);
    }

    public double getDist(){
        return Math.sqrt(x2*x2+y2*y2);
        //1.01x+1630 for single shooter
    }

    // Must be called else robot control will be off
    public void endAutoAlign(){
        isAligning=false;
        _alignFirstLoop = true;
        _alignIntegral = 0;
        _lastAlignPower = 0;
        _alignStableLoops = 0;
        _lastAlignErrorDeg = 180.0; // reset so stale error can't trigger premature auto-fire next press
        _alignDtTimer.reset();
    }

    /** Returns the camera Tx angle (degrees) to the goal tag. 0 = perfectly centered. */
    public double getAlignmentAngle() {
        return limelightHardware2Axis.getTxDegreesForId(targetTagId);
    }

    public boolean isAlignedToGoal() {
        double angle = getAlignmentAngle();
        return Math.abs(angle) <= ALIGN_TARGET_DEG && _alignStableLoops >= ALIGN_STABLE_LOOPS;
    }

    public boolean hasGoalTag() {
        return Math.abs(getAlignmentAngle()) < 120.0;
    }

    public double getLastAlignErrorDeg() {
        return _lastAlignErrorDeg;
    }

    public double getLastAlignPower() {
        return _lastAlignPower;
    }

    public int getAlignStableLoops() {
        return _alignStableLoops;
    }

    public void updateCameraPitch(){
        // Pitch only — yaw is fixed at 0.
        // Horizontal panning was removed: it caused a feedback loop where panning moved the goal
        // tag out of FOV, hasGoalTag() returned false, and the servo panned further in a cycle.
        // The robot body now rotates toward the goal (MegaTag fallback) instead of the camera.
        double dist = Math.sqrt(x3*x3 + y3*y3);
        double pitchAngle = Math.toDegrees(Math.atan(18.0 / dist));
        limelightHardware2Axis.setServoAngles(0.0, pitchAngle);

        if (SHOW_DASHBOARD_DEBUG) {
            telemetry.addData("camera pitch deg", String.format("%.1f", pitchAngle));
        }
    };

    // gives you amount to turn based on movement
    private double thetadt (){
        if(isBlue){
            return (y2*dxdt-x2*dydt)/(x2*x2+y2*y2);// partial differentiation of theta with respect to x and y
        }else{
            return -(y2*dxdt-x2*dydt)/(x2*x2+y2*y2);
        }

    }
    private double distdt(){
        if(isBlue){
            return x2+y2/Math.sqrt(x2*x2+y2*y2);
        }
        else{
            return x2+y2/Math.sqrt(x2*x2+y2*y2);
        }
    }
    private double calcDifference(double targetHeading){
        double difference = heading - targetHeading;

        // Normalize to [-π, π] range
        while (difference > Math.PI) {
            difference -= 2 * Math.PI;
        }
        while (difference < -Math.PI) {
            difference += 2 * Math.PI;
        }
        return difference;
    };

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Resets position of localizer based on camera
    // currently this is called in the loop of the opmode every 20 times
    public void reset(boolean reset){

        TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();
        Pose2D botPose = limelightHardware2Axis.getRobotPos(c); // is null when camera cannot tell position

        if(runningActions.isEmpty()){
            if(botPose!=null&&reset){
                if( Math.abs(botPose.getHeading(AngleUnit.RADIANS)-driveClass.localizer.getPose().heading.toDouble()) < Math.PI/16){
                    // TODO: why does it only reset if the error is above 4 inches
                    //TODO: fix it so camera always gets right position
                    if(Math.abs(botPose.getX(DistanceUnit.INCH)-driveClass.localizer.getPose().position.x)>4 || Math.abs(botPose.getY(DistanceUnit.INCH)-driveClass.localizer.getPose().position.y)>4){
                        driveClass.localizer.setPose(new Pose2d(new Vector2d(botPose.getX(DistanceUnit.INCH),botPose.getY(DistanceUnit.INCH)), driveClass.localizer.getPose().heading.toDouble()));

                    }
                }else{
                    driveClass.localizer.setPose(new Pose2d(new Vector2d(botPose.getX(DistanceUnit.INCH),botPose.getY(DistanceUnit.INCH)), botPose.getHeading(AngleUnit.RADIANS)));
                }
            }
        }
        if (SHOW_DASHBOARD_DEBUG) {
            Drawing.drawRobot(c, driveClass.localizer.getPose());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
