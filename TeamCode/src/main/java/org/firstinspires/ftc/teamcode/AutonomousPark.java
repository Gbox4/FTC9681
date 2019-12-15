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
    //CRServo claw1, claw2;
    CRServo drag1, drag2;
    //Servo pickUp1, pickUp2,
    //ModernRoboticsI2cRangeSensor SenseFront, SenseLeft, SenseRight,SenseFront
    timeState forward;
    private StateMachine machine;


    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();


    ArrayList<CRServo> servoDrag= new ArrayList<CRServo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos = new ArrayList <CRServo> ();
    //ArrayList<ModernRoboticsI2cRangeSensor> mrrs = new ArrayList<ModernRoboticsI2cRangeSensor>();


    @Override
    public void init() {

        frontRight=hardwareMap.dcMotor.get("front right");
        frontLeft=hardwareMap.dcMotor.get("front left");
        backRight=hardwareMap.dcMotor.get("back right");
        backLeft=hardwareMap.dcMotor.get("back left");

        //claw1=hardwareMap.crservo.get("claw 1");
        //claw2=hardwareMap.crservo.get("claw 2");
        extendArm=hardwareMap.dcMotor.get("extend arm");

        drag1= hardwareMap.crservo.get("drag front");
        drag2= hardwareMap.crservo.get ("drag back");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);

        servoDrag.add(drag1);
        servoDrag.add(drag2);

        //crServos.add(claw1);
        //crServos.add(claw2);

        /* ----- Code for Foundation Drag -------
        rightStrafe1 = new driveState(40, .5, motors, "strafeRight");
        down = new CRServoState (5000, .25, -.25, servoDrag);
        down1 = new CRServoState(5000, .25, -.25, servoDrag);
        nothing = new timeState(1000, 0, motors, "forward");
        nothing1 = new timeState(2000, 0, motors, "forward");

        leftStrafe1 = new driveState(40, .5, motors, "strafeLeft");

        backwards1 = new driveState(16, .5, motors, "backwards");
        up = new CRServoState(4000, -.5, .5, servoDrag); */

        forward = new timeState (1100, .5, motors, "forward"); //without encoders



        /* ----- Code for Foundation Drag -------
        rightStrafe1.setNextState(down); //goes to timeState if parking on inside only
        down.setNextState(down1);
        down1.setNextState(nothing);
        nothing.setNextState(nothing1);
        nothing1.setNextState(leftStrafe1);
        leftStrafe1.setNextState(backwards1);
        backwards1.setNextState(up);
        up.setNextState(timeState);
        */

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
