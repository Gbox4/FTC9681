package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.ArrayList;

import static java.lang.Thread.sleep;

@Autonomous(name = "FoundationRight", group = "Iterative OpMode")

public class AutonomousFoundationRight extends OpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    Servo drag1;
    Servo mrClamp;
    private StateMachine machine;
    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos = new ArrayList <CRServo> ();



    driveState strafeLeft;
    timeState towardsFoundation;
    oneServo lowerClamp;
    oneServo raiseClamp;
    driveState strafeRight;
    timeState forwardsFoundation1;
    timeState turnLeft;
    markerServoState down;




    @Override
    public void init() {

        if (true) {

            frontRight = hardwareMap.dcMotor.get("front right");
            frontLeft = hardwareMap.dcMotor.get("front left");
            backRight = hardwareMap.dcMotor.get("back right");
            backLeft = hardwareMap.dcMotor.get("back left");
            extendArm = hardwareMap.dcMotor.get("extend arm");

            drag1 = hardwareMap.servo.get("drag front");
            mrClamp = hardwareMap.servo.get("mrClamp");
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            motors.add(frontLeft);
            motors.add(frontRight);
            motors.add(backLeft);
            motors.add(backRight);

        }

        strafeLeft = new driveState(16, .3, motors, "strafeRight");
        towardsFoundation = new timeState (1100, .5, motors, "forward");
        lowerClamp = new oneServo(2100, 0.28, mrClamp);
        forwardsFoundation1 = new timeState(4000,  .4, motors, "backward");
        raiseClamp = new oneServo(1000, .7, mrClamp);
        strafeRight = new driveState(50,.5,motors,"strafeLeft");
        turnLeft = new timeState(1000, .5, motors, "turnLeft");
        down = new markerServoState(1000, -.5, drag1); //may need to change value, hardware changed servo position


        strafeLeft.setNextState(towardsFoundation);
        towardsFoundation.setNextState(lowerClamp);
        lowerClamp.setNextState(forwardsFoundation1);
        forwardsFoundation1.setNextState(raiseClamp);
        raiseClamp.setNextState(strafeRight);
        strafeRight.setNextState(turnLeft);
        turnLeft.setNextState(down);
        down.setNextState(null);



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
