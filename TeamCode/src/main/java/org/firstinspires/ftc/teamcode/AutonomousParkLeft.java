package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;
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
