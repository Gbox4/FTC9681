package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;


public class clampDriveBackState implements State {
    int newleftBackTarget;
    int newrightBackTarget;
    int  newleftFrontTarget;
    int  newrightFrontTarget;
    double distance;


    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    CRServo servo1, servo2;
    private double Power;
    private double Power2;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encod,er
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static double driveSpeed = 0.6;
    static final double TURN_SPEED = 0.5;

    private String Movement;

    private State NextState;

    public clampDriveBackState(double target, double speed, ArrayList<DcMotor> motor, String movement, double power, double power2, ArrayList<CRServo> CRServos) {

        driveSpeed = speed;
        distance = target;
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        Movement = movement;

        servo1 = CRServos.get(0);
        servo2 = CRServos.get(1);

        Power = power;
        Power2 = power2;

    }

    public void setNextState(State state) {
        NextState  = state;
    }

    @Override
    public void start() {

        //Reset the encoders back to zero for the next movement
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        rightBack.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);

        //Bring them back to using encoders
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Setting their target to their current encoder value (should be zero) to the amount of inches times the counts per inches

        newleftBackTarget = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newrightBackTarget = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newleftFrontTarget = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newrightFrontTarget = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

    }

    @Override
    public State update() {

        if(newleftBackTarget > leftBack.getCurrentPosition() && newrightBackTarget > rightBack.getCurrentPosition() && newleftFrontTarget > leftFront.getCurrentPosition() && newrightFrontTarget > rightFront.getCurrentPosition() ) {




            leftBack.setPower(-driveSpeed);
            leftFront.setPower(-driveSpeed);
            rightBack.setPower(-driveSpeed);
            rightFront.setPower(-driveSpeed);

            leftBack.setPower(driveSpeed);
            leftFront.setPower(driveSpeed);
            rightBack.setPower(driveSpeed);
            rightFront.setPower(driveSpeed);

            servo1.setPower(Power);
            servo2.setPower(Power2);

            return this;
        }else {
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            leftBack.setPower(0);

            servo1.setPower(0);
            servo2.setPower(0);
            return NextState;
        }
    }
}