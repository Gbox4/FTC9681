package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name ="TeleOp4", group = "TeleOP")
public class TeleOp4 extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor raiseArm2;
    DcMotor extendArm;
    CRServo claw1;
    CRServo claw2;
    Servo wrist;
    Servo mrClamp;
    boolean powerControl = false;
    double powerGiven =0;
    boolean clamp = false;
    int powerButton;
    Servo drag1;
    CRServo drag2;
    double wristAngle = 0;
    double clampPos=.7;
    boolean clamper=true;
    double draga = -0.3;
    Servo mrServo;
    double mrServox = .5;
    ColorSensor mrSensor;


    public void init() {
        //hardware map is for phone

        frontRight = hardwareMap.dcMotor.get("front right");
        frontLeft = hardwareMap.dcMotor.get("front left");
        backRight = hardwareMap.dcMotor.get("back right");
        backLeft = hardwareMap.dcMotor.get("back left");
        raiseArm2 = hardwareMap.dcMotor.get("raise arm 2");
        extendArm = hardwareMap.dcMotor.get("extend arm");
        claw1 = hardwareMap.crservo.get("claw 1");
        claw2 = hardwareMap.crservo.get("claw 2");
        drag1 = hardwareMap.servo.get("drag front");
        drag2 = hardwareMap.crservo.get("drag back");
        wrist=hardwareMap.servo.get("wrist");
        mrClamp = hardwareMap.servo.get("mrClamp");
        mrServo = hardwareMap.servo.get("mrServo");
        mrSensor = hardwareMap.colorSensor.get("mrSensor");
    }


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

        raiseArm2.setDirection(DcMotorSimple.Direction.FORWARD);



        //          -----GAME PAD 1-----

        //              ###SPEED BOOST###
        if(gamepad1.right_trigger>0.1){
            powerButton=1;
        }else{
            powerButton =2;
        }


        //              ###CAPSTONE SERVO###
        if (gamepad1.b/* && draga > -.5*/){
            draga  -= 0.01;
        }
        else if (gamepad1.a/* && draga< .43*/){
            draga += 0.01;
        }

        drag1.setPosition(draga);
        telemetry.addData("Drag servo pos should be", draga);
        telemetry.addData("what it atually is", drag1.getPosition());

        //             ###COLOR SENSOR SERVO###
        if (gamepad1.x && mrServox > -.5) {
            mrServox -=0.01;
        }
        else if (gamepad1.y && mrServox<.5){
            mrServox +=0.01;
        }

        mrServo.setPosition(mrServox);
        telemetry.addData("this is the mrServo", mrServo.getPosition());


        //             ###COLOR SENSOR###
        telemetry.addData("mrSensor values", mrSensor.alpha());


        //           ###FOUNDATION SERVO###
        if (gamepad1.right_bumper && clampPos>.28){
            clampPos  -= 0.01;
        }
        else if (gamepad1.left_bumper && clampPos<.90){
            clampPos += 0.01;
        }


        mrClamp.setPosition(clampPos);
        //telemetry.addData("clampPos = ", clampPos);
      //  telemetry.update();

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



        //              ###ARM EXTENSION###

        extendArm.setPower(-gamepad2.right_stick_y);

        //              ###WRIST###

        if (gamepad2.right_bumper && wristAngle>-0.7){
            wristAngle  -= 0.01;
        }
        else if (gamepad2.left_bumper && wristAngle<0.7){
            wristAngle += 0.01;
        }


    wrist.setPosition(wristAngle);


        //              ###ARM RAISING###

        raiseArm2.setPower(-gamepad2.left_stick_y);


       // telemetry.update();




    }


}

