package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name = "AutoTest", group = "Iterative OpMode")

public class AutoTest extends OpMode {
    //Motor Declarations
    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    CRServo claw1, claw2;
    ColorSensor cSensor;
    Servo cSensorServo;
    DcMotor raiseArmMotor;

    private StateMachine machine;

    //State machine declarations (In order of execution)
    oneServo sensorDown;
    driveState strafeLeft;
    ColorState colorState;
    timeState wait;
    timeState backwards;
    driveState strafeLeft2;
    extendArmState reachOut;
    CRServoState2 closeClamp;

    clampDriveState strafeRight;
    CRServoState2 closeClamp2;
    timeState wait2;
    timeState forward;

    extendArmState raiseArm;
    extendArmState reachOut2;
    CRServoState open;
    timeState backwards2;


    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos;

    {
        crServos = new ArrayList<CRServo>();
    }


    @Override
    public void init() {

        if (true) {

            frontRight = hardwareMap.dcMotor.get("front right");
            frontLeft = hardwareMap.dcMotor.get("front left");
            backRight = hardwareMap.dcMotor.get("back right");
            backLeft = hardwareMap.dcMotor.get("back left");

            //TODO: Change hardware maps
            cSensor = hardwareMap.colorSensor.get("mrSensor");
            cSensorServo = hardwareMap.servo.get("mrServo");
            raiseArmMotor = hardwareMap.dcMotor.get("raise arm 2");

            extendArm = hardwareMap.dcMotor.get("extend arm");

            claw1 = hardwareMap.crservo.get("claw 1");
            claw2 = hardwareMap.crservo.get("claw 2");

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            motors.add(frontLeft);
            motors.add(frontRight);
            motors.add(backLeft);
            motors.add(backRight);

            crServos.add(claw1);
            crServos.add(claw2);
        }

        sensorDown = new oneServo(500, 0, cSensorServo);
        strafeLeft = new driveState(30, .3, motors, "strafeLeft");
        colorState = new ColorState(motors, cSensor,"backward","alpha",4500);
        wait = new timeState(500, 0, motors, "backward");
        backwards = new timeState(500, .5, motors, "backward");
        strafeLeft2 = new driveState(16, .5, motors, "strafeLeft");
        reachOut = new extendArmState(1200, -.5, extendArm);
        closeClamp = new CRServoState2(1500,-1,1, crServos);
        strafeRight = new clampDriveState(18,.5, motors, "strafeRight", -1, 1, crServos);
        closeClamp2 = new CRServoState2(1500, -1, 1, crServos);
        wait2 = new timeState(500, 0, motors, "backward");
        forward = new timeState(0, .5, motors, "forward");

        //TODO: this time needs to be adjusted. also is power 1 or -1?
        raiseArm = new extendArmState(1000, 1, raiseArmMotor);
        reachOut2 = new extendArmState(2000, -.5, extendArm);
        open = new CRServoState(1000, 1, -1, crServos);
        backwards2 = new timeState(1500, .5, motors, "backward");




        raiseArm.setNextState(null);


    }
    @Override
    public void start(){


        machine = new StateMachine(raiseArm);
    }
    @Override
    public void loop()  {


        machine.update();

        if (colorState.done) {
            //TODO: Adjust this value based on distance from first block to bridge
            forward.Time = (int)colorState.totalTime+1500;
            telemetry.addData("colorTime:",colorState.totalTime);
            colorState.done = false;
        }
        telemetry.update();

    }

    @Override
    public void stop() {
    }


}


