package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import static java.lang.Thread.sleep;

@Autonomous(name = "AutoPark", group = "Iterative OpMode")

public class AutonomousPark extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;

    CRServo drag2;
    Servo drag1;

    timeState forward;
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
        drag2= hardwareMap.crservo.get ("drag back");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);

        servoDrag.add(drag2);

        forward = new timeState (1100, .5, motors, "forward"); //without encoders


    }
    @Override
    public void start(){


        machine = new StateMachine(forward); //start with rightStrafe1 for foundation, or parking on inside

    }
    @Override
    public void loop()  {


        machine.update();

    }

    @Override
    public void stop() {
    }


}
