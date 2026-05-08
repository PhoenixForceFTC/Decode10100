package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.autos.AutoActions;

public class MotifKicking {

    public enum Motif {
        PPG,
        PGP,
        GPP,
        INVALID;
    }

    public static Motif GameMotif = Motif.PPG;
    private RobotHardware hardware;

    private ElapsedTime myTimer = new ElapsedTime(2);

    private int[] shootingSequence;
    private boolean _simultaneous = false; // true = all 3 fire at once, false = sequential motif
    private int _seqStage = -1; // -1 = idle; 0-3 = active stage in sequential sequence

    public MotifKicking(RobotHardware hardware){
        this.hardware = hardware;
    }

    private static Motif AutoActionstoKickerKickMotif(AutoActions.Motif motif){
        if (motif == AutoActions.Motif.PPG) {
            return Motif.PPG;
        } else if  (motif == AutoActions.Motif.PGP) {
            return Motif.PGP;
        } else if (motif == AutoActions.Motif.GPP) {
            return Motif.GPP;
        }
        return Motif.INVALID;
    }

    public static void updateMotif(AutoActions.Motif motif){
        GameMotif = AutoActionstoKickerKickMotif(motif);
    }

    public static void updateMotif(LimelightHardware2Axis.Motif motif){
        if (motif == null) {
            return;
        }
        switch (motif) {
            case GPP: GameMotif = Motif.GPP; break;
            case PGP: GameMotif = Motif.PGP; break;
            case PPG: GameMotif = Motif.PPG; break;
        }
    }

    private static Motif colorsToMotif(Intake_Incomplete.BallColor[] colors){

        if (colors[0] == Intake_Incomplete.BallColor.GREEN &&
                colors[1] == Intake_Incomplete.BallColor.PURPLE &&
                colors[2] == Intake_Incomplete.BallColor.PURPLE){
            return Motif.GPP;
        } else if (colors[0] == Intake_Incomplete.BallColor.PURPLE &&
                colors[1] == Intake_Incomplete.BallColor.GREEN &&
                colors[2] == Intake_Incomplete.BallColor.PURPLE){
            return Motif.PGP;
        } else if (colors[0] == Intake_Incomplete.BallColor.PURPLE &&
                colors[1] == Intake_Incomplete.BallColor.PURPLE &&
                colors[2] == Intake_Incomplete.BallColor.GREEN){
            return Motif.PPG;
        }

        return Motif.INVALID;
    }

    private static int[] fireAutoKickerSeq(Motif targetMotif, Motif intakeMotif)
    {
        if(targetMotif == null || targetMotif == Motif.INVALID){targetMotif = Motif.GPP;}
        if(intakeMotif == null || intakeMotif == Motif.INVALID){intakeMotif = Motif.GPP;}

        char[] targetMotifSeq = targetMotif.toString().toCharArray();
        char[] intakeMotifSeq = intakeMotif.toString().toCharArray();
        int[] seqArr = new int[3];
        int count = 0;
        for (char c : targetMotifSeq) {
            int pos = findChar(intakeMotifSeq, c);
            if (pos>-1 && pos<3) {
                seqArr[count]=pos;
                count++;
                intakeMotifSeq[pos] = 'x';
            }
        }

        return seqArr;
    }

    private static int findChar(char[] arr, char target) {
        if (arr == null) {
            return -1; // handle null array
        }

        for (int i = 0; i < arr.length; i++) {
            if (arr[i] == target) {
                return i; // found, return index
            }
        }

        return -1; // not found
    }



    public void kickForMotifTeleOp(){

        Intake_Incomplete.BallColor leftColor =
                hardware.intake.detectBallSticky(hardware.intake._colorSensorLeft,
                        hardware.intake._distanceSensorLeft, 85,
                        hardware.intake._leftBallColor, "L");

        Intake_Incomplete.BallColor middleColor =
                hardware.intake.detectBallSticky(hardware.intake._colorSensorMiddle,
                        hardware.intake._distanceSensorMiddle, 45,
                        hardware.intake._middleBallColor, "M");

        Intake_Incomplete.BallColor rightColor =
                hardware.intake.detectBallSticky(hardware.intake._colorSensorRight,
                        hardware.intake._distanceSensorRight, 85,
                        hardware.intake._rightBallColor, "R");

        Intake_Incomplete.BallColor[] colors = {leftColor, middleColor, rightColor};

        Motif intakeMotif = colorsToMotif(colors);

        shootingSequence = fireAutoKickerSeq(GameMotif, intakeMotif);

        ElapsedTime timer = new ElapsedTime();

        hardware.kickers.fireKickerAuto(shootingSequence[0]);
        while(timer.seconds() < 0.4){}
        hardware.kickers.fireKickerAuto(shootingSequence[1]);
        while(timer.seconds() < 0.8){}
        hardware.kickers.fireKickerAuto(shootingSequence[2]);
        while(timer.seconds() < 1.2){}
        hardware.kickers.retractKickerAuto(0);
        hardware.kickers.retractKickerAuto(1);
        hardware.kickers.retractKickerAuto(2);

    }

    /**
     * Start the kick sequence.
     * @param simultaneous3Ball  true  = all 3 kickers fire at once (3-ball mode)
     *                           false = kickers fire one at a time per motif order (1-ball mode)
     */
    public void setKick(boolean simultaneous3Ball) {
        _simultaneous = simultaneous3Ball;
        if (simultaneous3Ball) {
            // Simultaneous 3-ball: reset all kicker timers at once.
            // runFinal() holds kicked position for KICKER_ACTION_DELAY then auto-retracts.
            hardware.kickers.fireAll3Simultaneous();
        } else {
            updateMotif(hardware.limelightHardware2Axis.updateStoredGameMotif(false));
            // Sequential motif: read ball colors and build firing order.
            Intake_Incomplete.BallColor leftColor =
                    hardware.intake.detectBallSticky(hardware.intake._colorSensorLeft,
                            hardware.intake._distanceSensorLeft, 85,
                            hardware.intake._leftBallColor, "L");
            Intake_Incomplete.BallColor middleColor =
                    hardware.intake.detectBallSticky(hardware.intake._colorSensorMiddle,
                            hardware.intake._distanceSensorMiddle, 45,
                            hardware.intake._middleBallColor, "M");
            Intake_Incomplete.BallColor rightColor =
                    hardware.intake.detectBallSticky(hardware.intake._colorSensorRight,
                            hardware.intake._distanceSensorRight, 85,
                            hardware.intake._rightBallColor, "R");

            Intake_Incomplete.BallColor[] colors = {leftColor, middleColor, rightColor};
            Motif intakeMotif = colorsToMotif(colors);
            shootingSequence = fireAutoKickerSeq(GameMotif, intakeMotif);
            _seqStage = 0; // start sequential sequence
        }
        if (myTimer == null) { myTimer = new ElapsedTime(); }
        myTimer.reset();
    }

    /** Defaults to sequential (1-ball) mode. */
    public void setKick() {
        setKick(false);
    }

    public void checkKick() {
        if (_simultaneous) {
            // Simultaneous mode: runFinal() handles kicker positions via timers.
            // Nothing extra needed here.
            return;
        }
        if (_seqStage < 0) return; // idle — nothing to do

        // Sequential mode: fire each kicker exactly once via fireKicker() so that
        // runFinal()'s timer-based position control holds the servo in kicked position
        // for KICKER_ACTION_DELAY (1.0s) before auto-retracting.
        // Stage 0: fire immediately; stages 1-2: fire at 0.4s intervals.
        double t = myTimer.time();
        if (_seqStage == 0) {
            hardware.kickers.fireKicker(shootingSequence[0]);
            _seqStage = 1;
        } else if (t >= 0.4 && _seqStage == 1) {
            hardware.kickers.fireKicker(shootingSequence[1]);
            _seqStage = 2;
        } else if (t >= 0.8 && _seqStage == 2) {
            hardware.kickers.fireKicker(shootingSequence[2]);
            _seqStage = 3;
        } else if (t >= 1.8 && _seqStage == 3) {
            // Last kicker fired at t=0.8s; KICKER_ACTION_DELAY=1.0s → retracted by t=1.8s.
            _seqStage = -1; // sequence complete
        }
    }

    /** Returns true while any kicker is still in motion (fired but not yet retracted). */
    public boolean isKicking() {
        if (_simultaneous) {
            return myTimer.time() < 1.0; // KICKER_ACTION_DELAY
        }
        return _seqStage >= 0;
    }
}
