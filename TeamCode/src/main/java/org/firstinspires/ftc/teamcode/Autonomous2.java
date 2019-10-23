package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


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

@Autonomous(name = "auto2", group = "Iterative OpMode")

public class Autonomous2 extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft;
   // Servo /*pickUp1, pickUp2,*/ drag1, drag2;
    //  ModernRoboticsI2cRangeSensor SenseFront, SenseLeft, SenseRight,SenseFront2;// not sure which ones will be used

    timeState ts;
    pickUpState pickUp;
  //  pickUpState drag;
  //  pickUpState drag3;
    private StateMachine machine;

    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();


    ArrayList<Servo> servoDrag= new ArrayList<Servo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    //ArrayList<ModernRoboticsI2cRangeSensor> mrrs = new ArrayList<ModernRoboticsI2cRangeSensor>();


    @Override
    public void init() {

        frontRight=hardwareMap.dcMotor.get("front right");
        frontLeft=hardwareMap.dcMotor.get("front left");
        backRight=hardwareMap.dcMotor.get("back right");
        backLeft=hardwareMap.dcMotor.get("back left");

        //  pickUp1=hardwareMap.servo.get("pickUp1");
        //   pickUp2=hardwareMap.servo.get("pickUp2");

     //   drag1= hardwareMap.servo.get("drag front");
     //   drag2= hardwareMap.servo.get ("drag back");
        /*SenseFront=hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sense 1");
        SenseLeft=hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sense 2");
        SenseRight=hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sense 3");
        SenseFront2=hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sense 4");
*/
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // servoPickUp.add(pickUp1);
        //servoPickUp.add(pickUp2);

    //    servoDrag.add(drag1);
    //    servoDrag.add(drag2);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);
/*
        mrrs.add(SenseFront);
        mrrs.add(SenseFront2);
        mrrs.add(SenseRight);
        mrrs.add(SenseLeft);
      */
        //  pickUp = new pickUpState(.5, .5, servoPickUp,1000);


        //tihs is for dragging.
    //    drag = new pickUpState(.5,.5, servoDrag,1000);
        ts = new timeState(800, .5, motors, "backward");
    //    drag3 = new pickUpState(-.5, .5, servoDrag, 1000);

    //    drag.setNextState(ts);
       // ts.setNextState(drag3);
     //   drag3.setNextState(null);



    }
    @Override
    public void start(){

        machine = new StateMachine(ts);

    }
    @Override
    public void loop()  {


        machine.update();

    }


    @Override
    public void stop() {
    }


}
