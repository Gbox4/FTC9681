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

@Autonomous(name = "notFoundationLeft", group = "Iterative OpMode")

public class notFoundationLeft extends OpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm, raiseArm;
    CRServo drag1, drag2;
    Servo mrClamp;
    private StateMachine machine;
    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();
    ArrayList<CRServo> servoDrag= new ArrayList<CRServo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos = new ArrayList <CRServo> ();
    ArrayList<CRServo> claws = new ArrayList <CRServo> ();
    CRServo claw1;
    CRServo claw2;

    timeState towardsStone;
    extendArmState extendClaw;
    extendArmState raiseArm1;
    CRServoState close;
    extendArmState retractClaw;
    extendArmState lowerArm;
    timeState backToWall;
    driveState strafeLeft;
    CRServoState open;
    oneServo down;
    driveState strafeRight;

    //clampDriveState dragToWall;


    @Override
    public void init() {

        if (true) {

            frontRight = hardwareMap.dcMotor.get("front right");
            frontLeft = hardwareMap.dcMotor.get("front left");
            backRight = hardwareMap.dcMotor.get("back right");
            backLeft = hardwareMap.dcMotor.get("back left");
            extendArm = hardwareMap.dcMotor.get("extend arm");
            raiseArm = hardwareMap.dcMotor.get("raise arm");
            claw1 = hardwareMap.crservo.get("claw 1");
            claw2 = hardwareMap.crservo.get("claw 2");
            drag1 = hardwareMap.crservo.get("drag front");
            drag2 = hardwareMap.crservo.get("drag back");
            mrClamp = hardwareMap.servo.get("mrClamp");

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            motors.add(frontLeft);
            motors.add(frontRight);
            motors.add(backLeft);
            motors.add(backRight);

            servoDrag.add(drag1);
            servoDrag.add(drag2);
            claws.add(claw1);
            claws.add(claw2);
        }

        towardsStone = new timeState(1100, .5, motors, "forward");
        raiseArm1 = new extendArmState(500, 1, raiseArm);
        extendClaw = new extendArmState(1000, .5, extendArm);
        close = new CRServoState(500, -1, 1, claws);
        lowerArm = new extendArmState(500, -1, raiseArm);
        retractClaw = new extendArmState(1000, -.5, extendArm);
        backToWall = new timeState (300, .5, motors, "backward");
        open = new CRServoState(500, 1, -1, claws);
        down = new oneServo(500, .37, mrClamp);

        towardsStone.setNextState(raiseArm1);
        raiseArm1.setNextState(extendClaw);
        extendClaw.setNextState(close);
        close.setNextState(lowerArm);
        lowerArm.setNextState(retractClaw);
        retractClaw.setNextState(backToWall);
        backToWall.setNextState(open);
        open.setNextState(down);
        down.setNextState(null);


    }
    @Override
    public void start(){


        machine = new StateMachine(towardsStone);

    }
    @Override
    public void loop()  {


        machine.update();

    }


    @Override
    public void stop() {
    }


}
