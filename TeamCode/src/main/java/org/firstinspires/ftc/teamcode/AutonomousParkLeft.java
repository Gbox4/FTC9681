package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.ArrayList;

import static java.lang.Thread.sleep;

@Autonomous(name = "AutoParkLeft", group = "Iterative OpMode")

public class AutonomousParkLeft extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    //CRServo claw1, claw2;
    CRServo drag1, drag2;
    // Servo /*pickUp1, pickUp2,*/ drag1, drag2;
    //  ModernRoboticsI2cRangeSensor SenseFront, SenseLeft, SenseRight,SenseFront2;// not sure which ones will be used

    driveState rightStrafe1;
    CRServoState down;
    clampDriveState clampDrive;
    timeState nothing;
    driveState leftStrafe1;
    timeState nothing1;
    driveState backwards1;
    CRServoState up;
    timeState timeState;
    timeState timeStrafeLeft;
    driveState forward1;
    clampDriveState leftDrive;
    clampDriveState backDrive;
    CRServoState down1;
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

        leftStrafe1 = new driveState(40, .5, motors, "strafeLeft");
        timeState = new timeState (1100, .5, motors, "forward");

        timeState.setNextState(leftStrafe1);
        leftStrafe1.setNextState(null);

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
