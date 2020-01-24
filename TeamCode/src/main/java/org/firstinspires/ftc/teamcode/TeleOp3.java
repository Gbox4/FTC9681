package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.Locale;

@TeleOp(name ="TeleOp3", group = "TeleOP")
public class TeleOp3 extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    ElapsedTime runtime;

    DcMotor raiseArm1;
    DcMotor raiseArm2;
    DcMotor extendArm;
    Servo claw1, claw2;
    CRServo wrist;
    boolean powerControl = false;
    double powerGiven =0;
    int powerButton;
    double pos1 = 0.9;

    double pos2 = 0.1;
    //CRServo drag1, drag2;
    boolean prevx = false;
    boolean prevy = false;
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    double armPowerMultiplier = 0.5;
    double clawIncrease = 0.1;

    boolean clamp = false;


    // DcMotor fan;
    // Servo marker, servoTouch, servoSlide, servoFlap;
    //Servo servoMin;
    //  CRServo servoSlide;


    // DcMotor lift;
    //  DcMotor MineralLifter;
   /* double mInt = 0.5;
    DigitalChannel touchSense;
    int powerInt = 2;
    boolean touch;
    ElapsedTime runTime;
    long lastCall = 0;
*/
    // public TeleOp1() {
    //    runTime = new ElapsedTime();
    // }

    public void init() {
        //hardware map is for phone

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //    touchSense = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        frontRight = hardwareMap.dcMotor.get("front right");
        frontLeft = hardwareMap.dcMotor.get("front left");
        //backRight = hardwareMap.dcMotor.get("back right");
        //backLeft = hardwareMap.dcMotor.get("back left");
        raiseArm1 = hardwareMap.dcMotor.get("raise arm 1");
        raiseArm2 = hardwareMap.dcMotor.get("raise arm 2");
        extendArm = hardwareMap.dcMotor.get("extend arm");
        claw1 = hardwareMap.servo.get("claw 1");
        claw2 = hardwareMap.servo.get("claw 2");
        wrist = hardwareMap.crservo.get("wrist");
        //wheels
        //drag1 = hardwareMap.crservo.get("drag front");
        //drag2 = hardwareMap.crservo.get("drag back");
        // pulley = hardwareMap.dcMotor.get("pulley"); //pulley for intake
     /*   fan = hardwareMap.dcMotor.get("fan");
        lift = hardwareMap.dcMotor.get("lift"); //lift mechanism
        MineralLifter = hardwareMap.dcMotor.get("mineralLifter");
        marker = hardwareMap.servo.get("marker"); //servo for team marker
        servoTouch = hardwareMap.servo.get("servoTouch");
        touchSense.setMode(DigitalChannel.Mode.INPUT);
        servoMin = hardwareMap.servo.get("servoMin");
        servoSlide = hardwareMap.servo.get("servoSlide");
        servoFlap = hardwareMap.servo.get("servoFlap");
        */


    }

    public void loop() {

        frontRight.setPower(gamepad1.right_stick_y/powerButton);
        frontLeft.setPower(gamepad1.left_stick_y/powerButton);


        //Assignment of motor power in relation to wheels
        //frontLeft.setPower(fLeftPower/powerButton);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);


        raiseArm1.setDirection(DcMotorSimple.Direction.REVERSE);
        raiseArm2.setDirection(DcMotorSimple.Direction.FORWARD);


        /*if(gamepad2.a){
            powerControl= true;
            powerGiven = gamepad2.left_stick_y/3;
            telemetry.addData("gamepad2.left stick y" , powerGiven);
            telemetry.update();

        }
        else if(gamepad2.b){
            powerControl = false;
        }*/
        // extendArm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

        //raiseArm.setPower(-gamepad1.right_trigger + gamepad1.left_trigger);

        // claw1.setPower(gamepad2.right_trigger);

        //  claw2.setPower(gamepad2.left_trigger);

        //open and close right claw
        /*if(gamepad2.right_trigger > 0){
            claw1.setPosition(-gamepad2.right_trigger); //opens right claw
        }
        else if(gamepad2.right_bumper){
            claw1.setPower(1); //closes right claw
        }else{
            claw1.setPower(0);
        }

        //open and close the left claw
        if (gamepad2.left_trigger>0){
            claw2.setPower(gamepad2.left_trigger); //close
        }
        else if (gamepad2.left_bumper){
            claw2.setPower(-1); //close

        }else{
            claw2.setPower(0);
        }*/

        //More buttons for drivers - claw servos go down together
      /*  if (gamepad2.x){
            claw1.setPosition(claw1.getPosition()+ clawIncrease);

        }
        else if (gamepad2.y){
            claw1.setPosition(claw1.getPosition()- clawIncrease);
        }


        if (gamepad2.right_bumper){
            claw2.setPosition(claw2.getPosition() - clawIncrease);
        }
        else if (gamepad2.left_bumper){
            claw2.setPosition(claw2.getPosition() + clawIncrease);
        }
        else{
            claw2.setPosition(claw2.getPosition());
        }*/

/*      if(gamepad2.x&& !prevx){
            prevx=true;
            pos1+= .1;
        }
        else{
            prevx=false;
        }

        if(gamepad2.y&& !prevy){
            prevy=true;
            pos1-= .1;
        }
        else{
            prevy=false;
        }*/


        if(gamepad2.left_bumper){
            pos2 += .1;
        }
        if(gamepad2.right_bumper){
            pos2 -= .1;
        }

      claw1.setPosition(pos2);
//      claw2.setPosition(pos2);


        extendArm.setPower(-gamepad2.right_stick_y); //extends cascading rail slides


        if(gamepad1.right_trigger>0.1){
            powerButton=1;
        }else{
            powerButton =2;
        }

        //to keep the arm in one place by maintaining one power, depending on whether or not a was pressed last
      /*  if(powerControl){
            raiseArm1.setPower(powerGiven);
            raiseArm2.setPower(powerGiven);
        }
        else{*/


        if (gamepad2.a){
            armPowerMultiplier = 0.5;
        }
        if (gamepad2.b){
            armPowerMultiplier = 0.2;
        }


        raiseArm1.setPower((gamepad2.left_stick_y*armPowerMultiplier));
        raiseArm2.setPower((gamepad2.left_stick_y*armPowerMultiplier));
        // }

        if (gamepad2.right_bumper){
            wrist.setPower(0.5);
        }
        else if (gamepad2.left_bumper){
            wrist.setPower(-0.5);
        }
        else {
            wrist.setPower(0);
        }

        //raiseArm.setPower(gamepad1)


    /*    if(gamepad1.a){
            drag1.setPosition(.5);

        }
        if(gamepad1.b){
            drag2.setPosition(.5);
        }
        if(gamepad1.y){
            drag1.setPosition(0);
        }
        if(gamepad1.x){
            drag2.setPosition(1);
        }*/


        telemetry.update();

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }






}

