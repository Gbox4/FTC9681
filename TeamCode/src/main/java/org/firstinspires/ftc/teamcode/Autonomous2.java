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
    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    CRServo claw1, claw2;
   // Servo /*pickUp1, pickUp2,*/ drag1, drag2;
    //  ModernRoboticsI2cRangeSensor SenseFront, SenseLeft, SenseRight,SenseFront2;// not sure which ones will be used

    timeState forward1;
    extendArmState extend;
    CRServoState close;
    extendArmState retract;
    timeState rightTurn;
    timeState forward2;
    CRServoState open;
    timeState backwards1;

    private StateMachine machine;

    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();


    ArrayList<Servo> servoDrag= new ArrayList<Servo>();
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

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);

        crServos.add(claw1);
        crServos.add(claw2);

        forward1 = new timeState(1000, .5, motors, "forward");
        extend = new extendArmState (3000, 1.0, extendArm);
        close = new CRServoState(1500, .5,.5,crServos);
        retract = new extendArmState(3000,-1.0,extendArm);
        rightTurn = new timeState(1000, .5, motors, "turnRight");

        forward1.setNextState(extend);
        extend.setNextState(close);
        retract.setNextState(rightTurn);
        



    }
    @Override
    public void start(){

        machine = new StateMachine(forward1);

    }
    @Override
    public void loop()  {


        machine.update();

    }


    @Override
    public void stop() {
    }


}
