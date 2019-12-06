package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.teamcode.pickUpState; //necessary
import org.firstinspires.ftc.teamcode.timeState; //necessar


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import java.util.Locale;

import static java.lang.Thread.sleep;

@Autonomous(name = "AutoFoundation123", group = "Linear OpMode")

public class AutonomousFoundation123 extends LinearOpMode {


    //equivalent of TimeState
    public void timeMove(int time, double power, ArrayList<DcMotor> motor, String movement) {
        //int Time = time;
        DcMotor leftFront = motor.get(0);
        DcMotor rightFront = motor.get(1);
        DcMotor leftBack = motor.get(2);
        DcMotor rightBack = motor.get(3);
        String Movement = movement;
        double Power = power;
        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();
        while (mRuntime.milliseconds() < time) {
            if (Movement == "forward") { //for some reason == worked
                leftFront.setPower(Power);
                rightFront.setPower(Power);
                leftBack.setPower(Power);
                rightBack.setPower(Power);
            }
            if (Movement == "backward") {
                leftFront.setPower(-Power);
                rightFront.setPower(-Power);
                leftBack.setPower(-Power);
                rightBack.setPower(-Power);
            }
            if (Movement == "turnLeft") {
                leftFront.setPower(-Power);
                rightFront.setPower(Power);
                leftBack.setPower(-Power);
                rightBack.setPower(Power);

            }
            if (Movement == "turnRight") {
                leftFront.setPower(Power);
                rightFront.setPower(-Power);
                leftBack.setPower(Power);
                rightBack.setPower(-Power);

            }

        }
        if (time <= mRuntime.milliseconds()) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            // return NextState;
        }
    }

    public void Drive(double target, double speed, ArrayList<DcMotor> motor, String movement) {
        int newleftBackTarget;
        int newrightBackTarget;
        int newleftFrontTarget;
        int newrightFrontTarget;
        double distance;

        DcMotor leftFront;
        DcMotor rightFront;
        DcMotor leftBack;
        DcMotor rightBack;

        final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        double driveSpeed = 0.6;
        final double TURN_SPEED = 0.5;

        String Movement;

        driveSpeed = speed;
        distance = target;
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        Movement = movement;

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

        while (newleftBackTarget > leftBack.getCurrentPosition() && newrightBackTarget > rightBack.getCurrentPosition() && newleftFrontTarget > leftFront.getCurrentPosition() && newrightFrontTarget > rightFront.getCurrentPosition()) {

            if (Movement == ("left")) {
                leftBack.setPower(driveSpeed);
                leftFront.setPower(-driveSpeed);
                rightBack.setPower(-driveSpeed);
                rightFront.setPower(driveSpeed);

            } else if (Movement == ("right")) {
                leftBack.setPower(-driveSpeed);
                leftFront.setPower(driveSpeed);
                rightBack.setPower(driveSpeed);
                rightFront.setPower(-driveSpeed);

            } else if (Movement == ("backwards")) {
                leftBack.setPower(-driveSpeed);
                leftFront.setPower(-driveSpeed);
                rightBack.setPower(-driveSpeed);
                rightFront.setPower(-driveSpeed);

            } else if (Movement == ("turnLeft")) {
                leftBack.setPower(-driveSpeed);
                leftFront.setPower(-driveSpeed);
                rightBack.setPower(driveSpeed);
                rightFront.setPower(driveSpeed);
            } else if (Movement == ("turnRight")) {
                leftBack.setPower(driveSpeed);
                leftFront.setPower(driveSpeed);
                rightBack.setPower(-driveSpeed);
                rightFront.setPower(-driveSpeed);
            } else if (Movement == ("strafeRight")) {
                leftBack.setPower(-driveSpeed);
                leftFront.setPower(driveSpeed);//negative
                rightBack.setPower(driveSpeed); //negative
                rightFront.setPower(-driveSpeed);
            } else if (Movement == ("strafeLeft")) {
                leftBack.setPower(driveSpeed);//neg
                leftFront.setPower(-driveSpeed);
                rightBack.setPower(-driveSpeed);
                rightFront.setPower(driveSpeed);//neg
            } else {

                leftBack.setPower(driveSpeed);
                leftFront.setPower(driveSpeed);
                rightBack.setPower(driveSpeed);
                rightFront.setPower(driveSpeed);
            }
        }
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }

    public void CrServoStuff(int time, double power, double power2, ArrayList<CRServo> CRServos) {
        int Time = time;
        CRServo servo1 = CRServos.get(0);
        CRServo servo2 = CRServos.get(1);

        double Power = power;
        double Power2 = power2;
        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        while (mRuntime.milliseconds() < Time) {

            servo1.setPower(Power);
            servo2.setPower(Power2);


        }
        if (Time <= mRuntime.milliseconds()) {
            servo1.setPower(0);
            servo2.setPower(0);

        }
    }


    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    //CRServo claw1, claw2;
    CRServo drag1, drag2;

    //ArrayList<Servo> servoPickUp = new ArrayList<Servo>();


    ArrayList<CRServo> servoDrag = new ArrayList<CRServo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos = new ArrayList<CRServo>();

    @Override
    public void runOpMode() {

        frontRight = hardwareMap.dcMotor.get("front right");
        frontLeft = hardwareMap.dcMotor.get("front left");
        backRight = hardwareMap.dcMotor.get("back right");
        backLeft = hardwareMap.dcMotor.get("back left");

        //claw1 = hardwareMap.crservo.get("claw 1");
        //claw2 = hardwareMap.crservo.get("claw 2");
        extendArm = hardwareMap.dcMotor.get("extend arm");

        drag1 = hardwareMap.crservo.get("drag front");
        drag2 = hardwareMap.crservo.get("drag back");


        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);

        servoDrag.add(drag1);
        servoDrag.add(drag2);

        //crServos.add(claw1);
        //crServos.add(claw2);

       /* rightStrafe1 = new driveState(40, .5, motors, "strafeRight");
        nothing = new timeState(1000, 0, motors, "forward");
        down = new CRServoState (4000, .25, -.25, servoDrag);

        leftStrafe1 = new driveState(40, .5, motors, "strafeLeft");
        nothing1 = new timeState(1000, 0, motors, "forward");
        backwards1 = new driveState(16, .5, motors, "backwards");
        nothing2 = new timeState(5000, 0, motors, "forward");
        up = new CRServoState(4000, -.5, .5, servoDrag);
        forward1 = new driveState(30, .5, motors, "forward");
        clampDrive = new clampDriveState(150,.5,motors,"strafeLeft",.5,-.5,servoDrag);
    */


        waitForStart();

        if(opModeIsActive()){
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            //Drive(1,.5,motors,"forward");
            timeMove(3000,.5 ,motors,"forward");
            //Drive(30, .9, motors, "forward");
             /* Drive(40, .5, motors, "strafeRight");
            CrServoStuff(2000, .5,-.5, servoDrag);
            drag1.setPower(.25);
            drag2.setPower(-.25);
            Drive(40, .5, motors, "strafeLeft");
            Drive (16, .5, motors, "backwards");
            CrServoStuff(2000, -.5, .5, servoDrag);
            Drive(30, .5, motors, "forward");

*/
             sleep(30000);
        }
    }
}