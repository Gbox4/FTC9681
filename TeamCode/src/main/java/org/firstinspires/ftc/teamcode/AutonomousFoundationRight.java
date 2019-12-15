package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.ArrayList;

import static java.lang.Thread.sleep;

@Autonomous(name = "AutoFoundationRight", group = "Iterative OpMode")

public class AutonomousFoundationRight extends OpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    CRServo drag1, drag2;
    private StateMachine machine;
    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();
    ArrayList<CRServo> servoDrag= new ArrayList<CRServo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos = new ArrayList <CRServo> ();



    driveState strafeLeft;
    driveState towardsFoundation;
    CRServoState lowerClamp;
    clampDriveState forwardsFoundation;
    CRServoState raiseClamp;
    driveState strafeRight;





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

        strafeLeft = new driveState(16, .3, motors, "strafeLeft"); //strafe left, lwoer target
        towardsFoundation = new driveState (28.5, .5, motors, "backwards");
        lowerClamp = new CRServoState(3000, .25, -.25, servoDrag);
        forwardsFoundation = new clampDriveState(28.5,.5,motors,"forwards",.5,-.5,servoDrag); //change to forwards
        raiseClamp = new CRServoState(3000, -.25,.25, servoDrag); //add lift clamp (CRServoState)
        strafeRight = new driveState(44,.5,motors,"strafeRight");//add rightstrafe



        strafeLeft.setNextState(towardsFoundation);
        towardsFoundation.setNextState(lowerClamp);
        lowerClamp.setNextState(forwardsFoundation);
        forwardsFoundation.setNextState(raiseClamp);
        raiseClamp.setNextState(strafeRight);
        strafeRight.setNextState(null);



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
