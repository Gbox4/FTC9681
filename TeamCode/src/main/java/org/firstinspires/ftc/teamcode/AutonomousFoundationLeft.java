package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;
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

@Autonomous(name = "AutoFoundationLeft", group = "Iterative OpMode")

public class AutonomousFoundationLeft extends OpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    CRServo drag1, drag2;
    private StateMachine machine;
    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();
    ArrayList<CRServo> servoDrag= new ArrayList<CRServo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos = new ArrayList <CRServo> ();



    driveState strafeLeft;
    CRServoState lowerClamp;
    clampDriveState turnFoundation;

    timeState wait;

    //clampDriveState dragToWall;




    @Override
    public void init() {

        if (true) {

            frontRight = hardwareMap.dcMotor.get("front right");
            frontLeft = hardwareMap.dcMotor.get("front left");
            backRight = hardwareMap.dcMotor.get("back right");
            backLeft = hardwareMap.dcMotor.get("back left");
            extendArm = hardwareMap.dcMotor.get("extend arm");

            drag1 = hardwareMap.crservo.get("drag front");
            drag2 = hardwareMap.crservo.get("drag back");

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            motors.add(frontLeft);
            motors.add(frontRight);
            motors.add(backLeft);
            motors.add(backRight);

            servoDrag.add(drag1);
            servoDrag.add(drag2);
        }

        strafeLeft = new driveState(40, .3, motors, "strafeRight"); //change to strafe left
        //add backwards
        lowerClamp = new CRServoState(3000, .25, -.25, servoDrag);
        //turnFoundation = new clampDriveState(70, .5, motors, "turnLeft", .5, -.5, servoDrag); //maybe no turn? Do not really need
        //dragToWall = new clampDriveState(70, .5, motors, "strafeLeft", .5, -.5, servoDrag); //change to forwards
        //lift clamp
        //right strafe
        //move backwards

        wait = new timeState(2000, 0, motors, "forward");

        strafeLeft.setNextState(lowerClamp);
        lowerClamp.setNextState(wait);
        wait.setNextState(turnFoundation);
        turnFoundation.setNextState(null);
        //dragToWall.setNextState(null);


    }
    @Override
    public void start(){


        machine = new StateMachine(strafeLeft);

    }
    @Override
    public void loop()  {


        machine.update();

    }


    @Override
    public void stop() {
    }


}