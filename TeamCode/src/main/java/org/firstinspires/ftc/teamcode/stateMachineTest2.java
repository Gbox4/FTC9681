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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

@Autonomous(name = "state", group = "Iterative OpMode")

public class stateMachineTest2 extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft;
    ModernRoboticsI2cRangeSensor SenseFront, SenseLeft, SenseRight,SenseFront2;// not sure which ones will be used
    rangeCalibrationState rangeState;
    distanceMoveState distanceState;
    timeState3 ts;
    timeState3 bs;
    timeState ashley;
    timeState erin;

    private StateMachine machine;

    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<ModernRoboticsI2cRangeSensor> mrrs = new ArrayList<ModernRoboticsI2cRangeSensor>();


    @Override
    public void init() {
        frontRight=hardwareMap.dcMotor.get("front right");
        frontLeft=hardwareMap.dcMotor.get("front left");
        backRight=hardwareMap.dcMotor.get("back left");
        backLeft=hardwareMap.dcMotor.get("back right");
        SenseFront=hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sense 1");
        SenseLeft=hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sense 2");
        SenseRight=hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sense 3");
        SenseFront2=hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sense 4");

        motors.add(frontLeft);
        motors.add(frontLeft);
        motors.add(backRight);
        motors.add(backLeft);

        mrrs.add(SenseFront);
        mrrs.add(SenseFront2);
        mrrs.add(SenseRight);
        mrrs.add(SenseLeft);



        ts = new timeState3  (3, .8, motors, "rightStrafe", "panda");
        bs = new timeState3 (5, .6, motors, "forward", "dog");
        ashley = new timeState (5,1, motors, "forward");
        erin = new timeState(5,1, motors, "turnRight");

        ashley.setNextState(erin);
        erin.setNextState(ts);
        ts.setNextState(bs);
        bs.setNextState(null);
    }
    @Override
    public void start(){

        Log.d("thing","10");
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
