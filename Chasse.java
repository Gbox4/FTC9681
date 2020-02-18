package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.hardware.Servo;
public class Chasse{
    private double move;
    private double crabwalk;
    private double rotation;
    private int control

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    public Chasse(){
        move=0;
        crabWalk=0;
        rotation=0;
        frontRight = hardwareMap.dcMotor.get("front right");
        frontLeft = hardwareMap.dcMotor.get("front left");
        backRight = hardwareMap.dcMotor.get("back right");
        backLeft = hardwareMap.dcMotor.get("back left");
    }
    public void Drive(){
        frontLeft.setPower(getFLeftPower()/);
        backLeft.setPower(getBLeftPower());
        frontRight.setPower(getFRightPower());
        backRight.setPower(getBRightPower());
        return;
    }
    private double getFLeftPower(){
         Update();
         double fLeftPower = Range.clip(move + rotation + crabWalk, -1.0, 1.0);
         return fLeftPower/control;
    }
    private double getBLeftPower(){
         Update();
         double BLeftPower = Range.clip(move + rotation - crabWalk, -1.0, 1.0);
         return BLeftPower/control;
    }
    private double getFRightPower(){
         Update();
         double fRightPower = Range.clip(move - rotation - crabWalk, -1.0, 1.0);
         return fRightPower/control;
    }
    private double getBRightPower(){
         Update();
         double BRightPower = Range.clip(move - rotation + crabWalk, -1.0, 1.0);
         return BRightPower/control;
    }
    private void Update(){
        UpdateControl();
        float move = -gamepad1.left_stick_y;
        float crabWalk = gamepad1.left_stick_x;
        float rotation = -gamepad1.right_stick_x;
        return;
    }
    private void UpdateControl(){
          if(gamepad1.right_trigger>0.1){
            control=1;
        }else{
            control =2;
        }
        return;
    }
}