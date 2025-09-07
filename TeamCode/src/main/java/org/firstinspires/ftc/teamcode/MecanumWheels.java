package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumWheels {
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftRearMotor;
    private DcMotorEx rightRearMotor;

    private LinearOpMode opMode;

    private boolean isSlowSpeed = false;

    //equivalent to an init() function that initiates the motors
    public MecanumWheels(LinearOpMode opMode){
        this.opMode = opMode;
        leftFrontMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "FL");
        rightFrontMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "FR");
        leftRearMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "BL");
        rightRearMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "BR");

        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(){
        move(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y, -opMode.gamepad1.right_stick_x*0.7);
    }

    public void move(double xMagnitude, double yMagnitude, double turn){

        if (opMode.gamepad1.x) {
            isSlowSpeed = false;
        } else if (opMode.gamepad1.a) {
            isSlowSpeed = true;
        }

        double magnitude = Math.hypot(xMagnitude, yMagnitude);
        double lFpower, rFpower, rRpower, lRpower;

        double angle = Math.atan2(-1 * yMagnitude, xMagnitude);
        double scale;

        /**rightFront and leftRear motors are controlled by magnitude = sin(angle - pi/4)
         * leftFront and rightRear motors are controlled by magnitude = sin(angle + pi/4)
         * For example, to move the robot at an angle of pi/4, you only want to move the rightFront and leftRear motors
         */

        lFpower = sin(angle + Math.PI/4) * magnitude - turn;
        rRpower = sin(angle + Math.PI/4) * magnitude + turn;

        rFpower = sin(angle - Math.PI/4) * magnitude + turn;
        lRpower = sin(angle - Math.PI/4) * magnitude - turn;
        //we have to subtract turn from the left side since the maps are a bit strange

        scale = Math.max(Math.abs(lFpower), (Math.max(Math.abs(rRpower),
                Math.max(Math.abs(rFpower), Math.max(Math.abs(lRpower), 1)))));

        if (isSlowSpeed) {
            scale *= 2;
        }

        //scale the values for turning since controller overflows at 1
        lFpower = lFpower/scale;
        rRpower = rRpower/scale;

        rFpower = rFpower/scale;
        lRpower = lRpower/scale;
        /**motor movements mapped to wheel movements:
         * leftFrontMotor forward - motor counterclockwise
         * rightFrontMotor forward - motor clockwise
         * leftRearMotor forward - motor counterclockwise
         * rightRearMotor forward - motor clockwise
         */

        leftFrontMotor.setPower(-lFpower);
        rightFrontMotor.setPower(rFpower);
        leftRearMotor.setPower(-lRpower);
        rightRearMotor.setPower(rRpower);

    }

    public void stop(){
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double lFPower = 0, rFpower = 0, rRpower = 0, lRpower = 0;

        leftFrontMotor.setPower(lFPower);
        rightFrontMotor.setPower(rFpower);
        leftRearMotor.setPower(lRpower);
        rightRearMotor.setPower(rRpower);
    }

    //moves forward one square
    public void moveForward() {
        double power = 0.5;
        leftFrontMotor.setPower(-power);
        leftRearMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);

        opMode.sleep(1000);

        stop();
    }

    public void turn90Clockwise(){
        move(0,0,-0.5);
        opMode.sleep(790);

        stop();
    }

    public void turn90CountClockwise(){
        move(0,0,0.5);
        opMode.sleep(790);

        stop();
    }

    public void moveForwardHalf() {
        double power = 0.5;
        leftFrontMotor.setPower(-power);
        leftRearMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);

        opMode.sleep(500);

        stop();
    }

    public void moveForwardLittle() {
        double power = 0.5;
        leftFrontMotor.setPower(-power);
        leftRearMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);

        opMode.sleep(75);

        stop();
    }

}
