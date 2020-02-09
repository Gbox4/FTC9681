package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.ArrayList;

import static java.lang.Thread.sleep;

@Autonomous(name = "AutoParkRight", group = "Iterative OpMode")

public class AutonomousParkRight extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    Servo drag1;


    driveState rightStrafe1;

    timeState timeState;
    private StateMachine machine;


    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();


    ArrayList<CRServo> servoDrag= new ArrayList<CRServo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos = new ArrayList <CRServo> ();


    @Override
    public void init() {

        frontRight=hardwareMap.dcMotor.get("front right");
        frontLeft=hardwareMap.dcMotor.get("front left");
        backRight=hardwareMap.dcMotor.get("back right");
        backLeft=hardwareMap.dcMotor.get("back left");

        extendArm=hardwareMap.dcMotor.get("extend arm");

        drag1= hardwareMap.servo.get("drag front");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);


        rightStrafe1 = new driveState(40, .5, motors, "strafeRight");
        timeState = new timeState (1100, .5, motors, "forward");

        timeState.setNextState(rightStrafe1);
        rightStrafe1.setNextState(null);



    }
    @Override
    public void start(){

        machine = new StateMachine(timeState);

    }
    @Override
    public void loop()  {


        machine.update();

    }


    @Override
    public void stop() {
    }


}
