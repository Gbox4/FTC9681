package org.firstinspires.ftc.teamcode;

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

@TeleOp(name ="TeleOp1", group = "TeleOP")
public class TeleOp1 extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor raiseArm;
    Servo extendArm;
    Servo claw1;
    Servo claw2;
   // Servo drag1, drag2;

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

    //    touchSense = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        frontRight = hardwareMap.dcMotor.get("front right");
        frontLeft = hardwareMap.dcMotor.get("front left");
        backRight = hardwareMap.dcMotor.get("back right");
        backLeft = hardwareMap.dcMotor.get("back left");
        raiseArm = hardwareMap.dcMotor.get("raise arm");
        extendArm = hardwareMap.servo.get("extend arm");
        claw1 = hardwareMap.servo.get("claw 1");
        claw2 = hardwareMap.servo.get("claw 2");
        //wheels
       // drag1 = hardwareMap.servo.get("drag front");
      //  drag2 = hardwareMap.servo.get("drag back");
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
        //In place of motor power, gamestick position is used determined by the controller

        float move = -gamepad1.left_stick_y;
        float rotation = -gamepad1.right_stick_x;
        float crabWalk = -gamepad1.left_stick_x;

        //Wheels: Holonomic drive formula uses values of gamestick position to move
        double fLeftPower = Range.clip(move + rotation + crabWalk, -1.0, 1.0);
        double bLeftPower = Range.clip(move + rotation - crabWalk, -1.0, 1.0);
        double fRightPower = Range.clip(move - rotation - crabWalk, -1.0, 1.0);
        double bRightPower = Range.clip(move - rotation + crabWalk, -1.0, 1.0);

        //Assignment of motor power in relation to wheels
        frontLeft.setPower(fLeftPower);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setPower(bLeftPower );
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setPower(fRightPower );

        backRight.setPower(bRightPower);

        extendArm.setPower(gamepad1.right_trigger-gamepad1.left_trigger)
        


        //raiseArm.setPower(gamepad1.)


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




        //pulley.setPower(gamepad2.left_stick_y);

      /*  if (gamepad1.right_trigger > .5) {
            telemetry.addData("Right trigger", gamepad1.right_trigger);
            powerInt = 1;
        } else {
            powerInt = 2;
        }
        if (gamepad2.right_bumper) {
            MineralLifter.setPower(-.5);

        } else if (gamepad2.left_bumper) {
            MineralLifter.setPower(.25);

        } else {
            MineralLifter.setPower(0);
        }
        if (gamepad2.y) {

            fan.setPower(-1);
        } else if (gamepad2.b) {
            fan.setPower(1);
        } else {
            fan.setPower((0));
        }
        if (gamepad2.x) {
            marker.setPosition(0);
        } else {
            marker.setPosition(1); //so if you hit x then it will lift the mech holding the marker up to place the marker
            //the position 0 is normal and just means you haven't pushed the button
        }

        //move the railslides out - use mInt to set to certain positions (negative to positive) depending on how many times dpad_up is pressed
        if (gamepad1.dpad_up) {

            servoSlide.setPosition(1.0);
        }
        //move the railslides back in - same idea as up, just in the other direction (positive to negative)
        else if (gamepad1.dpad_down){
            servoSlide.setPosition(0);
        }
        else {
            servoSlide.setPosition(.5);
        }
        if (gamepad1.dpad_left){
            servoFlap.setPosition(1);
        }

        if (gamepad1.dpad_right){
            servoFlap.setPosition(.4);
        }

        if(gamepad1.x){
            servoMin.setPosition(-.5);
        }
        else if (gamepad1.y) {
            servoMin.setPosition(.45);
        }
        else if (gamepad1.b) {
            servoMin.setPosition(1);
        }
        if (touchSense.getState() == true)
            telemetry.addData("groundTOuch", "is this working? it should be true " + touchSense.getState());
        else
            telemetry.addData("groundTOuch", "is this working? it should be false" + touchSense.getState());

        lift.setPower(-gamepad2.right_stick_y);
//        telemetry.addData("status", "Run Time:", runTime.toString());
        telemetry.addData("front Left Pos", frontLeft.getCurrentPosition());
        telemetry.addData("front Right Pos", frontRight.getCurrentPosition());
        telemetry.addData("back Left Pos", backLeft.getCurrentPosition());
        telemetry.addData("back Right Pos", backRight.getCurrentPosition());
        telemetry.addData("current touch pos", servoTouch.getPosition());
        telemetry.update();
        servoTouch.setPosition(1);
*/
    }




}

