package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class MotifKicking {
    /*Pseudocodish
    *
    * detect balls
    * find ball-motif
    * find kicking order
    * kick balls
    * */

    public enum Motif {
        PPG,
        PGP,
        GPP,
        INVALID;
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



    public void kickForMotifTeleOp(RobotHardware hardware, Motif targetMotif){

        Intake_Incomplete.BallColor leftColor =
                hardware.intake.detectBallSticky(hardware.intake._colorSensorLeft,
                        hardware.intake._distanceSensorLeft, 85,
                        hardware.intake._leftBallColor);

        Intake_Incomplete.BallColor middleColor =
                hardware.intake.detectBallSticky(hardware.intake._colorSensorMiddle,
                        hardware.intake._distanceSensorMiddle, 45,
                        hardware.intake._middleBallColor);

        Intake_Incomplete.BallColor rightColor =
                hardware.intake.detectBallSticky(hardware.intake._colorSensorRight,
                        hardware.intake._distanceSensorRight, 85,
                        hardware.intake._rightBallColor);

        Intake_Incomplete.BallColor[] colors = {leftColor, middleColor, rightColor};

        Motif intakeMotif = colorsToMotif(colors);

        int[] shootingSequence = fireAutoKickerSeq(targetMotif, intakeMotif);

        ElapsedTime timer = new ElapsedTime();

        hardware.kickers.fireKicker(shootingSequence[0]);
        while(timer.seconds() < 0.4){}
        hardware.kickers.fireKicker(shootingSequence[1]);
        while(timer.seconds() < 0.8){}
        hardware.kickers.fireKicker(shootingSequence[2]);

    }
}
