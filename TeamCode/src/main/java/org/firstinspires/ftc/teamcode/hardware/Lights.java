package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lights {
    //region --- Constants ---
    //--- Blink Speeds (in seconds)
    private static final double BLINK_FAST_INTERVAL = 0.15;
    private static final double BLINK_SLOW_INTERVAL = 0.5;

    //--- Pulse Pattern Settings
    private static final double PULSE_SOLID_DURATION = 2.0;
    //endregion

    //region --- Enums ---
    public enum Color
    {
        OFF(0.0),
        RED(0.29),
        ORANGE(0.33),
        YELLOW(0.38),
        GREEN(0.51),
        BLUE(0.61),
        PURPLE(0.69),
        WHITE(1.0);

        private final double position;

        Color(double position)
        {
            this.position = position;
        }

        public double getPosition()
        {
            return position;
        }
    }

    public enum Blink
    {
        NONE,
        SLOW,
        FAST,
        PULSE
    }
    //endregion

    //region --- Hardware ---
    private final Servo _servoLightLeft;
    private final Servo _servoLightMiddle;
    private final Servo _servoLightRight;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    private final int _robotVersion;
    //endregion

    //region --- Blink State ---
    private Color _leftColor = Color.OFF;
    private Color _middleColor = Color.OFF;
    private Color _rightColor = Color.OFF;

    private Blink _leftBlink = Blink.NONE;
    private Blink _middleBlink = Blink.NONE;
    private Blink _rightBlink = Blink.NONE;

    //--- Shared timers so all lights blink in sync
    private ElapsedTime _timerSlow = new ElapsedTime();
    private ElapsedTime _timerFast = new ElapsedTime();
    private boolean _slowOn = true;
    private boolean _fastOn = true;

    //--- Pulse pattern state (blink X times, then solid for Y seconds)
    private ElapsedTime _timerPulse = new ElapsedTime();
    private int _pulseBlinkCount = 0;
    private int _pulseBlinkTarget = 4;
    private boolean _pulseOn = true;
    private boolean _pulseInSolidPhase = false;

    //--- Test mode state
    private double _testPosition = 0.0;
    //endregion

    //region --- Constructor ---
    public Lights(
            Servo servoLightLeft,
            Servo servoLightMiddle,
            Servo servoLightRight,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry,
            int robotVersion,
            boolean showInfo
    )
    {
        this._servoLightLeft = servoLightLeft;
        this._servoLightMiddle = servoLightMiddle;
        this._servoLightRight = servoLightRight;
        this._gamepad1 = gamepad1;
        this._gamepad2 = gamepad2;
        this._telemetry = telemetry;
        this._robotVersion = robotVersion;
        this._showInfo = showInfo;

    }
    //endregion

    //region --- Initialize ---
    public void initialize()
    {
        //--- Set default light colors on initialization
        setLeft(Color.RED);
        setMiddle(Color.ORANGE);
        setRight(Color.YELLOW);
    }
    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Update shared timers for SLOW blink
        if (_timerSlow.seconds() >= BLINK_SLOW_INTERVAL)
        {
            _slowOn = !_slowOn;
            _timerSlow.reset();
        }

        //--- Update shared timers for FAST blink
        if (_timerFast.seconds() >= BLINK_FAST_INTERVAL)
        {
            _fastOn = !_fastOn;
            _timerFast.reset();
        }

        //--- Update PULSE pattern state
        if (_pulseInSolidPhase)
        {
            //--- In solid phase, wait for duration then restart blinking
            if (_timerPulse.seconds() >= PULSE_SOLID_DURATION)
            {
                _pulseInSolidPhase = false;
                _pulseBlinkCount = 0;
                _pulseOn = true;
                _timerPulse.reset();
            }
        }
        else
        {
            //--- In blink phase
            if (_timerPulse.seconds() >= BLINK_SLOW_INTERVAL)
            {
                _pulseOn = !_pulseOn;
                _timerPulse.reset();

                //--- Count each full blink cycle (on->off counts as 1)
                if (!_pulseOn)
                {
                    _pulseBlinkCount++;
                    if (_pulseBlinkCount >= _pulseBlinkTarget)
                    {
                        //--- Done blinking, switch to solid phase
                        _pulseInSolidPhase = true;
                        _pulseOn = true;
                        _timerPulse.reset();
                    }
                }
            }
        }

        //--- Handle left light blinking
        if (_leftBlink != Blink.NONE)
        {
            boolean isOn = getBlinkState(_leftBlink);
            _servoLightLeft.setPosition(isOn ? _leftColor.getPosition() : Color.OFF.getPosition());
        }

        //--- Handle middle light blinking
        if (_middleBlink != Blink.NONE)
        {
            boolean isOn = getBlinkState(_middleBlink);
            _servoLightMiddle.setPosition(isOn ? _middleColor.getPosition() : Color.OFF.getPosition());
        }

        //--- Handle right light blinking
        if (_rightBlink != Blink.NONE)
        {
            boolean isOn = getBlinkState(_rightBlink);
            _servoLightRight.setPosition(isOn ? _rightColor.getPosition() : Color.OFF.getPosition());
        }
    }

    //--- Helper to get the current on/off state for a blink mode
    private boolean getBlinkState(Blink blink)
    {
        switch (blink)
        {
            case SLOW:
                return _slowOn;
            case FAST:
                return _fastOn;
            case PULSE:
                return _pulseOn;
            default:
                return true;
        }
    }
    //endregion

    //region --- Public Methods - Set All Lights ---

    //--- Set all lights to the same color enum
    public void setAll(Color color)
    {
        setLeft(color);
        setMiddle(color);
        setRight(color);
    }

    //endregion

    //region --- Public Methods - Set Individual Lights by Color Enum ---

    public void setLeft(Color color)
    {
        setLeft(color, Blink.NONE);
    }

    public void setLeft(Color color, Blink blink)
    {
        _leftColor = color;
        _leftBlink = blink;
        if (blink == Blink.NONE)
        {
            _servoLightLeft.setPosition(color.getPosition());
        }
    }

    public void setLeft(Color color, int pulseCount)
    {
        _leftColor = color;
        _leftBlink = Blink.PULSE;
        setPulseCount(pulseCount);
    }

    public void setMiddle(Color color)
    {
        setMiddle(color, Blink.NONE);
        _telemetry.addData("Now I set the middle light color", color);
    }

    public void setMiddle(Color color, Blink blink)
    {
        _middleColor = color;
        _middleBlink = blink;
        if (blink == Blink.NONE)
        {
            _servoLightMiddle.setPosition(color.getPosition());
            _telemetry.addData("I set the middle light color light position", color.getPosition());
        }
    }

    public void setMiddle(Color color, int pulseCount)
    {
        _middleColor = color;
        _middleBlink = Blink.PULSE;
        setPulseCount(pulseCount);
    }

    public void setRight(Color color)
    {
        setRight(color, Blink.NONE);
    }

    public void setRight(Color color, Blink blink)
    {
        _rightColor = color;
        _rightBlink = blink;
        if (blink == Blink.NONE)
        {
            _servoLightRight.setPosition(color.getPosition());
        }
    }

    public void setRight(Color color, int pulseCount)
    {
        _rightColor = color;
        _rightBlink = Blink.PULSE;
        setPulseCount(pulseCount);
    }

    //endregion

    //region --- Public Methods - Set All Lights with Blink ---

//    public void setAll(Color color, Blink blink)
//    {
//        setLeft(color, blink);
//        setMiddle(color, blink);
//        setRight(color, blink);
//    }

    public void setAll(Color color, int pulseCount)
    {
        setPulseCount(pulseCount);
        _leftColor = color;
        _leftBlink = Blink.PULSE;
        _middleColor = color;
        _middleBlink = Blink.PULSE;
        _rightColor = color;
        _rightBlink = Blink.PULSE;
    }

    //--- Set the pulse count and reset the pulse state
    private void setPulseCount(int count)
    {
        _pulseBlinkTarget = count + 1;
        _pulseBlinkCount = 0;
        _pulseOn = true;
        _pulseInSolidPhase = false;
        _timerPulse.reset();
    }

    //endregion

    //region --- Public Methods - Convenience Color Methods ---

    public void setAllRed() { setAll(Color.RED); }
    public void setAllYellow() { setAll(Color.YELLOW); }
    public void setAllGreen() { setAll(Color.GREEN); }
    public void setAllBlue() { setAll(Color.BLUE); }
    public void setAllPurple() { setAll(Color.PURPLE); }
    public void setAllWhite() { setAll(Color.WHITE); }
    public void setAllOff() { setAll(Color.OFF); }

    //endregion

    //region --- Test Mode ---

    //--- Use gamepad2 dpad up/down to adjust light position value
    //--- Displays current position in telemetry for finding color values
    public void testColors()
    {
        if (_gamepad2.dpad_up)
        {
            _testPosition += 0.01;
         //   sleep(200);
        }
        else if (_gamepad2.dpad_down)
        {
            _testPosition -= 0.01;
         //   sleep(200);
        }

        //--- Clamp value between 0 and 1
        _testPosition = Range.clip(_testPosition, 0.0, 1.0);

        //--- Set all lights to test position
        _servoLightLeft.setPosition(_testPosition);
        _servoLightMiddle.setPosition(_testPosition);
        _servoLightRight.setPosition(_testPosition);

        _telemetry.addData("Light Test", "%4.2f", _testPosition);
    }

    //--- Use gamepad2 Y/B/A/X buttons to test different light patterns
    //--- Y: Green Purple Green (solid)
    //--- B: Purple Green Purple (slow blink)
    //--- A: Blue Red Yellow (fast blink)
    //--- X: Blue (solid) Red (solid) Yellow (fast blink)
//    public void testPattern()
//    {
//        if (_gamepad2.y)
//        {
//            //--- Pattern 1: Green Purple Green (solid)
//            setLeft(Color.GREEN);
//            setMiddle(Color.PURPLE);
//            setRight(Color.GREEN);
//            _telemetry.addData("Light Pattern", "Y: Green Purple Green (solid)");
//        }
//        else if (_gamepad2.b)
//        {
//            //--- Pattern 2: Purple Green Purple (slow blink)
//            setLeft(Color.PURPLE, Blink.SLOW);
//            setMiddle(Color.GREEN, Blink.SLOW);
//            setRight(Color.PURPLE, Blink.SLOW);
//            _telemetry.addData("Light Pattern", "B: Purple Green Purple (slow)");
//        }
//        else if (_gamepad2.a)
//        {
//            //--- Pattern 3: Blue Red Yellow (fast blink)
//            setLeft(Color.BLUE, Blink.FAST);
//            setMiddle(Color.RED, Blink.FAST);
//            setRight(Color.YELLOW, Blink.FAST);
//            _telemetry.addData("Light Pattern", "A: Blue Red Yellow (fast)");
//        }
//        else if (_gamepad2.x)
//        {
//            //--- Pattern 4: Blue (solid) Red (solid) Yellow (fast blink)
//            setLeft(Color.BLUE);
//            setMiddle(Color.RED);
//            setRight(Color.YELLOW, 3);
//            _telemetry.addData("Light Pattern", "X: Blue Red Yellow(fast)");
//        }
//    }
//    private void sleep(long milliseconds)
//    {
//        try { Thread.sleep(milliseconds); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
//    }



    //endregion

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Light Left", "%4.2f", _servoLightLeft.getPosition());
            _telemetry.addData("Light Middle", "%4.2f", _servoLightMiddle.getPosition());
            _telemetry.addData("Light Right", "%4.2f", _servoLightRight.getPosition());
        }
    }

    //endregion
}
