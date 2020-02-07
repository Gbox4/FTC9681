package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.ArrayList;

@Autonomous(name = "AutoColorStoneRight", group = "Iterative OpMode")

public class ColorStoneRight extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    //CRServo claw1, claw2;
    CRServo claw1, claw2;
    //Servo pickUp1, pickUp2,
    //ModernRoboticsI2cRangeSensor SenseFront, SenseLeft, SenseRight,SenseFront
    ColorSensor mrSensor;
    Servo mrServo;

    timeState forward;
    driveState strafeLeft;
    extendArmState reachOut;
    private StateMachine machine;
    clampDriveState strafeRight;
    timeState turnLeft;
    CRServoState2 close;
    CRServoState open;
    CRServoState2 close2;
    timeState park;
    ColorState colorState;
    ColorState colorState2;
    timeState backwards;
    timeState backwards2;
    driveState strafeRightAgain;


    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();

    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos;

    {
        crServos = new ArrayList<CRServo>();
    }
    //ArrayList<ModernRoboticsI2cRangeSensor> mrrs = new ArrayList<ModernRoboticsI2cRangeSensor>();


    @Override
    public void init() {

        frontRight=hardwareMap.dcMotor.get("front right");
        frontLeft=hardwareMap.dcMotor.get("front left");
        backRight=hardwareMap.dcMotor.get("back right");
        backLeft=hardwareMap.dcMotor.get("back left");
        mrSensor=hardwareMap.colorSensor.get("mrSensor");
        mrServo=hardwareMap.servo.get("mrServo");

        //claw1=hardwareMap.crservo.get("claw 1");
        //claw2=hardwareMap.crservo.get("claw 2");
        extendArm=hardwareMap.dcMotor.get("extend arm");

        claw1= hardwareMap.crservo.get("claw 1");
        claw2 = hardwareMap.crservo.get("claw 2");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);

        crServos.add(claw1);
        crServos.add(claw2);

        //orward = new timeState (1100, .5, motors, "forward"); //without encoders
        strafeLeft = new driveState(28, .3, motors, "strafeLeft");
        colorState = new ColorState(motors, mrSensor,"backward","alpha",4500);
        backwards = new timeState(1000, .5, motors, "backward");
        colorState2 = new ColorState(motors, mrSensor,"forward","alpha",800);
        backwards2 = new timeState(400, .5, motors, "backward");
        strafeRightAgain = new driveState(16, .5, motors, "strafeLeft");
        reachOut = new extendArmState(1200, -.5, extendArm);
        close = new CRServoState2(1500,-1,1, crServos);
        strafeRight = new clampDriveState(18,.5, motors, "strafeRight", -1, 1, crServos);
        close2 = new CRServoState2(1500, -1, 1, crServos);
        //turnLeft = new timeState(2000, .5, motors, "turnLeft");
        forward = new timeState(1500, .5, motors, "forward");
        open = new CRServoState(1000, 1, -1, crServos);
        park = new timeState (700, .5, motors, "backward");



        //strafe right 18
        //180 degree turn
        //forward 2500
        //let go


        strafeLeft.setNextState(colorState);
        colorState.setNextState(backwards);
        backwards.setNextState(colorState2);
        colorState2.setNextState(backwards2);
        backwards2.setNextState(strafeRightAgain);
        strafeRightAgain.setNextState(reachOut);
        reachOut.setNextState(close);
        close.setNextState(strafeRight);
        strafeRight.setNextState(close2);
       // close2.setNextState(turnLeft);
        //turnLeft.setNextState(forward);
        close2.setNextState(forward);
        forward.setNextState(open);
        open.setNextState(park);
        park.setNextState(null);



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

