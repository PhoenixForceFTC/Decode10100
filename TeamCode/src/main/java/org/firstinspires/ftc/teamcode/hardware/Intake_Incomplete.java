package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//endregion

@Config
public class Intake_Incomplete
{




    //region --- Hardware ---
    private final DcMotorEx _intake;
    FtcDashboard _dashboard = FtcDashboard.getInstance();

    private double intakePower = 0.5;


    private final Gamepad _gamepad;
    private final Gamepad _gamepad2;
    public final Telemetry _telemetry;

    //--- Ball Detection Settings (per-sensor distance thresholds)
    //--- Each sensor may have different mounting distances
    private static final double LEFT_DISTANCE_THRESHOLD_MM = 85.0;    // Left sensor: ball if < 80mm
    private static final double MIDDLE_DISTANCE_THRESHOLD_MM = 45.0;  // middle sensor: ball if < 80mm
    private static final double RIGHT_DISTANCE_THRESHOLD_MM = 85.0;   // Right sensor: ball if < 80mm

    //--- Averaging settings
    private static final int AVERAGING_SAMPLES = 5; //Number of samples to average

    private static final double GREEN_RATIO_THRESHOLD = 2.5; // G/R must exceed this for green
    private static final double PURPLE_BG_THRESHOLD = 0.85; // G/R must exceed this for purple
    private static final double PURPLE_GR_MAX = 1.8; // G/R must be below this for purple
    private static int loopCount = 0;

    //region --- Enums ---
    public enum BallColor
    {
        NONE,
        GREEN,
        PURPLE,
        UNKNOWN  // Ball detected but color unclear
    }
    //endregion

    //--- Color Sensors (cast to DistanceSensor for distance reading)
    public ColorSensor _colorSensorLeft = null;
    public ColorSensor _colorSensorMiddle = null;
    public ColorSensor _colorSensorRight = null;  // Future use
    public DistanceSensor _distanceSensorLeft = null;
    public DistanceSensor _distanceSensorMiddle = null;
    public DistanceSensor _distanceSensorRight = null;  // Future use

    //--- Lights reference for ball indication
    private Lights _lights = null;

    //endregion

    //region --- State ---
   // private boolean _intakeOn = false;

    //--- Ball detection state
    public BallColor _leftBallColor = BallColor.NONE;
    public BallColor _middleBallColor = BallColor.NONE;
    public BallColor _rightBallColor = BallColor.NONE;

    //--- Sensor averaging buffers (circular buffers for last N readings)
    private int _leftBufferIndex = 0;

    private int _middleBufferIndex = 0;

    private double[] _rightDistBuffer = new double[AVERAGING_SAMPLES];
    private int[] _rightRedBuffer = new int[AVERAGING_SAMPLES];
    private int[] _rightGreenBuffer = new int[AVERAGING_SAMPLES];
    private int[] _rightBlueBuffer = new int[AVERAGING_SAMPLES];
    private int _rightBufferIndex = 0;

    //--- Averaged values (for telemetry display)
    private double _leftAvgDist, _leftAvgR, _leftAvgG, _leftAvgB;
    private double _middleAvgDist, _middleAvgR, _middleAvgG, _middleAvgB;
    private double _rightAvgDist, _rightAvgR, _rightAvgG, _rightAvgB;

    //--- Sticky color flags (for telemetry)
    private boolean _leftIsSticky = false;
    private boolean _middleIsSticky = false;
    private boolean _rightIsSticky = false;
    //endregion

    //region --- Constructor
    public Intake_Incomplete( DcMotorEx intake, Gamepad gamepad,Gamepad gamepad2, Telemetry telemetry, boolean showInfo)
    {
        this._intake = intake;
        this._intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this._intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this._gamepad = gamepad;
        this._gamepad2 = gamepad2;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
    }
    //endregion

    //region --- Movement ---
    public void forward(){
        _intake.setPower(intakePower);
    } //--- Outward movement of artifacts

    public void backwardFast(double speed) {_intake.setPower(-speed);}

    public void backward(){
        _intake.setPower(-intakePower);
    } //--- Inward movement

    public void backward_slow(){_intake.setPower(-intakePower);}

    public void stop(){
        _intake.setPower(0);
    }
    //endregion

    //--- Uses controls to control intake, outtake, and stop
    public void run(){
        // change controls later
        if (_gamepad2.left_trigger>0.2){
            forward();
        }
        if (_gamepad2.right_trigger>0.2){
            backward();
            restoreLights();
        }
        if (_gamepad.y){
            stop();
            restoreLights();
        }
        if (_gamepad2.y && _gamepad2.left_stick_y > 0.5){
            intakePower = Math.min(1, intakePower + 0.005);
        } else if (_gamepad2.y && _gamepad2.left_stick_y < -0.5){
            intakePower = Math.max(0, intakePower - 0.005);
        }

        detectBalls();
//        _telemetry.addData("I detected the left ball", _leftBallColor);
//        _telemetry.addData("I detected the middle ball", _middleBallColor);
//        _telemetry.addData("I detected the right ball", _rightBallColor);
        //_telemetry.addData("Left distance is", "%d", _leftAvgDist);

        updateLights();

        loopCount++;
        _telemetry.addData("Loop count:", "%d", loopCount);
        _telemetry.addData("Intake Power: ", intakePower);
        //--- Only detect balls and update lights while intaking or outtaking
    }

    public void setColorSensors(ColorSensor left, ColorSensor middle, ColorSensor right)
    {
        _colorSensorLeft = left;
        _colorSensorMiddle = middle;
        _colorSensorRight = right;

        //--- REV Color Sensor V3 also implements DistanceSensor
        _distanceSensorLeft = (DistanceSensor) left;
        _distanceSensorMiddle = (DistanceSensor) middle;
        _distanceSensorRight = (DistanceSensor) right;
    }

    public void setLights(Lights lights)
    {
        _lights = lights;
    }



    //region --- Ball Detection ---

    private void detectBalls()
    {
        //--- Update sensor buffers and detect balls using averaged values
        //--- Apply "sticky" color logic: only upgrade from UNKNOWN to a color, never downgrade
        _leftBallColor = detectBallSticky(
                _colorSensorLeft, _distanceSensorLeft, LEFT_DISTANCE_THRESHOLD_MM,
                _leftBallColor);
        _leftBufferIndex = (_leftBufferIndex + 1) % AVERAGING_SAMPLES;

        _middleBallColor = detectBallSticky(
                _colorSensorMiddle, _distanceSensorMiddle, MIDDLE_DISTANCE_THRESHOLD_MM,
                _middleBallColor);
        _middleBufferIndex = (_middleBufferIndex + 1) % AVERAGING_SAMPLES;

        _rightBallColor = detectBallSticky(
                _colorSensorRight, _distanceSensorRight, RIGHT_DISTANCE_THRESHOLD_MM,
                _rightBallColor);
        _rightBufferIndex = (_rightBufferIndex + 1) % AVERAGING_SAMPLES;
    }

    public BallColor detectBallSticky(
            ColorSensor colorSensor, DistanceSensor distanceSensor, double distanceThreshold, BallColor previousColor){

        int AveragingSamples = 10;

        if(colorSensor == null || distanceSensor == null){
            return BallColor.NONE;
        }

        double avgRed = 0, avgGreen = 0, avgBlue = 0, avgDist = 0;
        for(int i = 0; i < AveragingSamples; i++){
            double distance = distanceSensor.getDistance(DistanceUnit.MM);
            avgRed += colorSensor.red();
            avgBlue += colorSensor.blue();
            avgGreen += colorSensor.green();
            avgDist += Double.isNaN(distance) ? 999.0 : distance;
        }
        avgRed /= AveragingSamples;
        avgBlue /= AveragingSamples;
        avgGreen /= AveragingSamples;
        avgDist /= AveragingSamples;

        double greenToRed = (avgRed != 0 ? avgGreen/avgRed : 0);
        double blueToGreen = (avgGreen != 0 ? avgBlue/avgGreen : 0);

        if (avgDist > distanceThreshold)
        {
            return BallColor.NONE;
        }

        if (blueToGreen > PURPLE_BG_THRESHOLD && greenToRed < PURPLE_GR_MAX)
        {
            if(colorSensor == _colorSensorLeft){
                _leftBallColor = BallColor.PURPLE;
            }else if(colorSensor == _colorSensorMiddle){
                _middleBallColor = BallColor.PURPLE;
            }else if(colorSensor == _colorSensorRight){
                _rightBallColor = BallColor.PURPLE;
            }
            return BallColor.PURPLE;
        }

        if (greenToRed > GREEN_RATIO_THRESHOLD)
        {
            if(colorSensor == _colorSensorLeft){
                _leftBallColor = BallColor.GREEN;
            }else if(colorSensor == _colorSensorMiddle){
                _middleBallColor = BallColor.GREEN;
            }else if(colorSensor == _colorSensorRight){
                _rightBallColor = BallColor.GREEN;
            }
            return BallColor.GREEN;
        }

        if (previousColor == BallColor.GREEN || previousColor == BallColor.PURPLE)
        {
            return previousColor;  // Keep the last good color
        }

        //--- No previous good color, show UNKNOWN (red light)
        return BallColor.UNKNOWN;
    }

    private void setStickyFlag(String sensorName, boolean isSticky)
    {
        if (sensorName.equals("left")) _leftIsSticky = isSticky;
        else if (sensorName.equals("middle")) _middleIsSticky = isSticky;
        else if (sensorName.equals("right")) _rightIsSticky = isSticky;
    }

    //--- Calculate hue (0-360) from RGB values
    private double calculateHue(int r, int g, int b)
    {
        double red = r / 255.0;
        double green = g / 255.0;
        double blue = b / 255.0;

        double max = Math.max(red, Math.max(green, blue));
        double min = Math.min(red, Math.min(green, blue));
        double delta = max - min;

        if (delta == 0)
        {
            return 0;  // Achromatic (gray)
        }

        double hue;
        if (max == red)
        {
            hue = 60 * (((green - blue) / delta) % 6);
        }
        else if (max == green)
        {
            hue = 60 * (((blue - red) / delta) + 2);
        }
        else
        {
            hue = 60 * (((red - green) / delta) + 4);
        }

        if (hue < 0)
        {
            hue += 360;
        }

        return hue;
    }

    //endregion

    //region --- Light Updates ---

    private void updateLights()
    {
        if (_lights == null)
        {
            return;
        }

        //--- Update left light based on left ball color
        _lights.setLeft(ballColorToLightColor(_leftBallColor));
        //--- Update middle light based on middle ball color
        _lights.setMiddle(ballColorToLightColor(_middleBallColor));
        _telemetry.addData("I set the middle light color", ballColorToLightColor(_middleBallColor));
        //--- Update right light based on right ball color
        _lights.setRight(ballColorToLightColor(_rightBallColor));
        _telemetry.addData("I set the right light color", ballColorToLightColor(_rightBallColor));

    }

    private Lights.Color ballColorToLightColor(BallColor ballColor)
    {
        switch (ballColor)
        {
            case GREEN:
                return Lights.Color.GREEN;
            case PURPLE:
                return Lights.Color.PURPLE;
            case UNKNOWN:
                return Lights.Color.RED;
            case NONE:
                return Lights.Color.WHITE;
            default:
                return Lights.Color.WHITE;
        }
    }

    //--- Restore lights to default state (call when intake stops)
    public void restoreLights()
    {
        if (_lights != null)
        {
            _lights.initialize();  // Reset to default colors
        }
    }

    //endregion

    //--- Get detected ball colors


    //region --- Testing ---

    //--- Test color sensors - shows telemetry for tuning
    //--- Always runs detection and updates lights so testing works
    //--- regardless of intake state
    public void testColorSensors()
    {
        //--- Always detect balls and update lights in test mode
        detectBalls();
        updateLights();

        //--- Show detailed telemetry
        _telemetry.addLine("=== COLOR SENSOR TEST (5-sample avg) ===");

        if (_colorSensorLeft != null && _distanceSensorLeft != null)
        {
            int r = _colorSensorLeft.red();
            int g = _colorSensorLeft.green();
            int b = _colorSensorLeft.blue();
            double dist = _distanceSensorLeft.getDistance(DistanceUnit.MM);

            //--- Calculate ratios from averaged values
            double gToR = (_leftAvgR > 0) ? _leftAvgG / _leftAvgR : 0;
            double bToG = (_leftAvgG > 0) ? _leftAvgB / _leftAvgG : 0;

            _telemetry.addLine(String.format("--- LEFT SENSOR (max %.0fmm) ---", LEFT_DISTANCE_THRESHOLD_MM));
            _telemetry.addData("  Raw RGB", "R:%d G:%d B:%d", r, g, b);
            _telemetry.addData("  Avg RGB", "R:%.0f G:%.0f B:%.0f", _leftAvgR, _leftAvgG, _leftAvgB);
            _telemetry.addData("  G/R ratio", "%.2f (GREEN if >%.1f)", gToR, GREEN_RATIO_THRESHOLD);
            _telemetry.addData("  B/G ratio", "%.2f (PURPLE if >%.2f & G/R<%.1f)", bToG, PURPLE_BG_THRESHOLD, PURPLE_GR_MAX);
            _telemetry.addData("  Distance", "%.1f mm (avg %.1f)", dist, _leftAvgDist);
            _telemetry.addData("  Ball", "%s%s", _leftBallColor, _leftIsSticky ? " (STICKY)" : "");
        }
        else
        {
            _telemetry.addData("Left Sensor", "NOT CONNECTED");
        }

        if (_colorSensorMiddle != null && _distanceSensorMiddle != null)
        {
            int r = _colorSensorMiddle.red();
            int g = _colorSensorMiddle.green();
            int b = _colorSensorMiddle.blue();
            double dist = _distanceSensorMiddle.getDistance(DistanceUnit.MM);

            //--- Calculate ratios from averaged values
            double gToR = (_middleAvgR > 0) ? _middleAvgG / _middleAvgR : 0;
            double bToG = (_middleAvgG > 0) ? _middleAvgB / _middleAvgG : 0;

            _telemetry.addLine(String.format("--- middle SENSOR (max %.0fmm) ---", MIDDLE_DISTANCE_THRESHOLD_MM));
            _telemetry.addData("  Raw RGB", "R:%d G:%d B:%d", r, g, b);
            _telemetry.addData("  Avg RGB", "R:%.0f G:%.0f B:%.0f", _middleAvgR, _middleAvgG, _middleAvgB);
            _telemetry.addData("  G/R ratio", "%.2f (GREEN if >%.1f)", gToR, GREEN_RATIO_THRESHOLD);
            _telemetry.addData("  B/G ratio", "%.2f (PURPLE if >%.2f & G/R<%.1f)", bToG, PURPLE_BG_THRESHOLD, PURPLE_GR_MAX);
            _telemetry.addData("  Distance", "%.1f mm (avg %.1f)", dist, _middleAvgDist);
            _telemetry.addData("  Ball", "%s%s", _middleBallColor, _middleIsSticky ? " (STICKY)" : "");
        }
        else
        {
            _telemetry.addData("middle Sensor", "NOT CONNECTED");
        }

        if (_colorSensorRight != null && _distanceSensorRight != null)
        {
            int r = _colorSensorRight.red();
            int g = _colorSensorRight.green();
            int b = _colorSensorRight.blue();
            double dist = _distanceSensorRight.getDistance(DistanceUnit.MM);

            //--- Calculate ratios from averaged values
            double gToR = (_rightAvgR > 0) ? _rightAvgG / _rightAvgR : 0;
            double bToG = (_rightAvgG > 0) ? _rightAvgB / _rightAvgG : 0;

            _telemetry.addLine(String.format("--- RIGHT SENSOR (max %.0fmm) ---", RIGHT_DISTANCE_THRESHOLD_MM));
            _telemetry.addData("  Raw RGB", "R:%d G:%d B:%d", r, g, b);
            _telemetry.addData("  Avg RGB", "R:%.0f G:%.0f B:%.0f", _rightAvgR, _rightAvgG, _rightAvgB);
            _telemetry.addData("  G/R ratio", "%.2f (GREEN if >%.1f)", gToR, GREEN_RATIO_THRESHOLD);
            _telemetry.addData("  B/G ratio", "%.2f (PURPLE if >%.2f & G/R<%.1f)", bToG, PURPLE_BG_THRESHOLD, PURPLE_GR_MAX);
            _telemetry.addData("  Distance", "%.1f mm (avg %.1f)", dist, _rightAvgDist);
            _telemetry.addData("  Ball", "%s%s", _rightBallColor, _rightIsSticky ? " (STICKY)" : "");
        }
        else
        {
            _telemetry.addData("Right Sensor", "NOT CONNECTED");
        }

        _telemetry.addLine("");
        _telemetry.addData("Detection", "GREEN: G/R>%.1f | PURPLE: B/G>%.2f & G/R<%.1f",
                GREEN_RATIO_THRESHOLD, PURPLE_BG_THRESHOLD, PURPLE_GR_MAX);
    }

    //endregion

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
          //  _telemetry.addData("Intake Power", "%4.2f", _intake.getPower());
          //  _telemetry.addData("Intake On", _intakeOn);
          //  _telemetry.addData("Outtake Active", _outtakeActive);

            //--- Ball detection telemetry
            _telemetry.addData("Ball Left", _leftBallColor);
            _telemetry.addData("Ball middle", _middleBallColor);
            _telemetry.addData("Ball Right", _rightBallColor);

            //--- Raw sensor values for debugging
            if (_colorSensorLeft != null && _distanceSensorLeft != null)
            {
                _telemetry.addData("Left RGB", "R:%d G:%d B:%d",
                        _colorSensorLeft.red(), _colorSensorLeft.green(), _colorSensorLeft.blue());
                _telemetry.addData("Left Dist", "%.1f mm",
                        _distanceSensorLeft.getDistance(DistanceUnit.MM));
            }
            if (_colorSensorMiddle != null && _distanceSensorMiddle != null)
            {
                _telemetry.addData("middle RGB", "R:%d G:%d B:%d",
                        _colorSensorMiddle.red(), _colorSensorMiddle.green(), _colorSensorMiddle.blue());
                _telemetry.addData("middle Dist", "%.1f mm",
                        _distanceSensorMiddle.getDistance(DistanceUnit.MM));
            }
        }
        _telemetry.addData("motor power: ", _intake.getPower());
    }

    //endregion
    private final Boolean _showInfo;
    //--- Displays telemetry data


    //--- Gets speed of intake
    public double getSpeed(){
        return(_intake.getVelocity());
    }



}