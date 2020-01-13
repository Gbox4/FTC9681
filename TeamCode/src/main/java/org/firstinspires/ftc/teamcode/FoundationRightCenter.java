package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.ArrayList;

import static java.lang.Thread.sleep;

@Autonomous(name = "FoundationRightCenter", group = "Iterative OpMode")

public class FoundationRightCenter extends OpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    CRServo drag1, drag2;
    private StateMachine machine;
    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();
    ArrayList<CRServo> servoDrag= new ArrayList<CRServo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos = new ArrayList <CRServo> ();



    driveState strafeLeft;
    timeState towardsFoundation;
    CRServoState2 lowerClamp;
    clampDriveState forwardsFoundation;
    CRServoState raiseClamp;
    driveState strafeRight;
    timeState backward;
    driveState strafeRight2;
    timeState forwardsFoundation1;




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

        strafeLeft = new driveState(16, .3, motors, "strafeLeft");
        towardsFoundation = new timeState (1000, .5, motors, "backward"); //may need to change time
        lowerClamp = new CRServoState2(2100, -.5, .5, servoDrag);
        forwardsFoundation1 = new timeState(3000,  .5, motors, "forward");

        //forwardsFoundation = new clampDriveState(28.5,.5,motors,"forwards",-.5,.5,servoDrag);
        raiseClamp = new CRServoState(1000, .25,-.25, servoDrag);
        strafeRight = new driveState(32,.5,motors,"strafeRight");
        backward = new timeState(1000,.5,motors,"backward");
        strafeRight2 =new driveState(16,.5,motors,"strafeRight");


        strafeLeft.setNextState(towardsFoundation);
        towardsFoundation.setNextState(lowerClamp);
        lowerClamp.setNextState(forwardsFoundation1);
        forwardsFoundation1.setNextState(raiseClamp);
        raiseClamp.setNextState(strafeRight);


        strafeRight.setNextState(backward);
        backward.setNextState(strafeRight2);
        strafeRight2.setNextState(null);



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
