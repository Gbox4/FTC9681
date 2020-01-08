package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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


@TeleOp(name ="TeleOp4", group = "TeleOP")
public class TeleOp4 extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    //DcMotor raiseArm1;
    DcMotor raiseArm2;
    DcMotor extendArm;
    CRServo claw1;
    CRServo claw2;
    CRServo wrist;
    boolean powerControl = false;
    double powerGiven =0;
    boolean clamp = false;
    int powerButton;
    CRServo drag1, drag2;
    double wristAngle = 0;

    //double armPowerMultiplier = 0.5;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    public void init() {
        //hardware map is for phone

        //    touchSense = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        frontRight = hardwareMap.dcMotor.get("front right");
        frontLeft = hardwareMap.dcMotor.get("front left");
        backRight = hardwareMap.dcMotor.get("back right");
        backLeft = hardwareMap.dcMotor.get("back left");
        //raiseArm1 = hardwareMap.dcMotor.get("raise arm 1");
        raiseArm2 = hardwareMap.dcMotor.get("raise arm 2");
        extendArm = hardwareMap.dcMotor.get("extend arm");
        claw1 = hardwareMap.crservo.get("claw 1");
        claw2 = hardwareMap.crservo.get("claw 2");
        drag1 = hardwareMap.crservo.get("drag front");
        drag2 = hardwareMap.crservo.get("drag back");
        wrist = hardwareMap.crservo.get("wrist");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }

   /* private void setRaiseArmPower(float armPower, double multiplier){
        raiseArm1.setPower(armPower*multiplier);
        raiseArm2.setPower(armPower*multiplier);
        return;
    }*/

    public void loop() {
        //              -----STICK VARIABLES-----
        //For driving
        float move = -gamepad1.left_stick_y;
        float crabWalk = gamepad1.left_stick_x;
        float rotation = -gamepad1.right_stick_x;

        //For arm raising
        float rawRaiseValue = -gamepad2.left_stick_y;




        //              -----WHEEL LOGIC-----
        //Wheels: Holonomic drive formula uses values of gamestick position to move
        double fLeftPower = Range.clip(move + rotation + crabWalk, -1.0, 1.0);
        double bLeftPower = Range.clip(move + rotation - crabWalk, -1.0, 1.0);
        double fRightPower = Range.clip(move - rotation - crabWalk, -1.0, 1.0);
        double bRightPower = Range.clip(move - rotation + crabWalk, -1.0, 1.0);
        //Assignment of motor power in relation to wheels
        frontLeft.setPower(fLeftPower/powerButton);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setPower(bLeftPower/powerButton);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setPower(fRightPower/powerButton);

        backRight.setPower(bRightPower/powerButton);

        //raiseArm1.setDirection(DcMotorSimple.Direction.FORWARD);
        raiseArm2.setDirection(DcMotorSimple.Direction.FORWARD);



        //          -----GAME PAD 1-----

        //              ###SPEED BOOST###
        if(gamepad1.right_trigger>0.1){
            powerButton=1;
        }else{
            powerButton =2;
        }

        //              ###DRAG SERVOS###
        if(gamepad1.x){
            drag1.setPower(.5);
        }
        else if(gamepad1.b){
            drag1.setPower(-.5);
        }
        else{
            drag1.setPower(0);
        }
        if(gamepad1.a){
            drag2.setPower(.5);
        }
        else if(gamepad1.y){
            drag2.setPower(-.5);
        }
        else{
            drag2.setPower(0);
        }



        //          -----GAME PAD 2-----

        //              ###CLAMPS###
        if (gamepad2.x){
            clamp = true;
        }
        if (gamepad2.y){
            claw1.setPower(1);
            claw2.setPower(-1);
            clamp = false;
        }
        else if (!clamp){
            claw1.setPower(0);
            claw2.setPower(0);
        }
        if (clamp){
            claw1.setPower(-1);
            claw2.setPower(1);
        }



        //claw1: 1=open, 0=closed
        //claw2: 0=open, 1=closed

        /*//open
        if (gamepad2.y){
            claw1.setPosition(0.6);
            claw2.setPosition(0.4);
        }
        //close
       else if (gamepad2.x){
            claw1.setPosition(0.4);
            claw2.setPosition(0.6);
        }*/



        //              ###ARM EXTENSION###

        extendArm.setPower(-gamepad2.right_stick_y);

        //              ###WRIST###

        if (gamepad2.right_bumper && wristAngle>-0.7){
            wristAngle  -= 0.01;
        }
        else if (gamepad2.left_bumper && wristAngle<0.7){
            wristAngle += 0.01;
        }



        wrist.setPower(wristAngle);


        //              ###ARM RAISING###

        raiseArm2.setPower(gamepad2.left_stick_y);
/*
        // Fast raise arm mode
        if (gamepad2.right_trigger>0){
            //If the driver is trying to move the arm up:
            if (rawRaiseValue > 0) {
                setRaiseArmPower(rawRaiseValue, 0.6);
            }
            //If the driver is trying to move the arm down:
            else if (rawRaiseValue < 0) {
                setRaiseArmPower(0.1f, 0.35);
            }
            //If the driver is not moving the arm
            /*else {
                setRaiseArmPower(0.23f, 1);
            }
        }
        // Slow raise arm mode
     /*   else {
            //If the driver is trying to move the arm up:
            if (rawRaiseValue > 0) {
                setRaiseArmPower(rawRaiseValue, 0.35);
            }
            //If the driver is trying to move the arm down:
            else if (rawRaiseValue < 0) {
                setRaiseArmPower(0f, 1);
            }
            //If the driver is not moving the arm
           /* else {
                setRaiseArmPower(0.23f, 1);
            }
        }*/

        telemetry.update();




    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
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

