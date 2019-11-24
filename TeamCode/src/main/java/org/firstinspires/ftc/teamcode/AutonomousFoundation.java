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

@Autonomous(name = "AutoFoundation", group = "Iterative OpMode")

public class AutonomousFoundation extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    CRServo claw1, claw2;
    CRServo drag1, drag2;
    // Servo /*pickUp1, pickUp2,*/ drag1, drag2;
    //  ModernRoboticsI2cRangeSensor SenseFront, SenseLeft, SenseRight,SenseFront2;// not sure which ones will be used

    driveState rightStrafe1;
    CRServoState down;
    timeState nothing;
    driveState leftStrafe1;
    timeState nothing1;
    driveState backwards1;
    CRServoState up;
    timeState nothing2;
    driveState forward1;

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

        claw1=hardwareMap.crservo.get("claw 1");
        claw2=hardwareMap.crservo.get("claw 2");
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

        crServos.add(claw1);
        crServos.add(claw2);

        rightStrafe1 = new driveState(32, .6, motors, "strafeRight");
        nothing = new timeState(4000, 0, motors, "forward");
        down = new CRServoState (4000, .25, -.25, servoDrag);

        leftStrafe1 = new driveState(30, .5, motors, "strafeLeft");
        nothing1 = new timeState(1000, 0, motors, "forward");
        backwards1 = new driveState(16, .5, motors, "backwards");
        nothing2 = new timeState(5000, 0, motors, "forward");
        up = new CRServoState(1000, -.5, .5, servoDrag);
        forward1 = new driveState(30, .5, motors, "forward");

        rightStrafe1.setNextState(down);
        down.setNextState(leftStrafe1);
        leftStrafe1.setNextState(null);

        /*rightStrafe1.setNextState(down);
        down.setNextState(null);*/

        /*nothing.setNextState(down);
        down.setNextState(leftStrafe1);
        leftStrafe1.setNextState(nothing1);
        nothing1.setNextState(backwards1);
        backwards1.setNextState(up);
        up.setNextState(nothing2);
        nothing2.setNextState(forward1);
        forward1.setNextState(null);*/

        //leftStrafe1.setNextState(null);





    }
    @Override
    public void start(){

        //machine = new StateMachine(down);
        machine = new StateMachine(rightStrafe1);

    }
    @Override
    public void loop()  {


        machine.update();

    }


    @Override
    public void stop() {
    }


}

