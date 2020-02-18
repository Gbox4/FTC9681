package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.hardware.Servo;


public class Claws(){
     CRServo claw1;
     CRServo claw2;
     boolean clamp;
     

     public Claws(){
          claw1 = hardwareMap.crservo.get("claw 1");
          claw2 = hardwareMap.crservo.get("claw 2");
          clamp = false;
          
     }
     public void Use(){
     if(clamp){
         claw1.setPower(-1);
         claw2.setPower(1);
     }
     else{
       claw1.setPower(0);
       claw2.setPower(0);  
     }
     }
    public void Open(){
        claw1.setPower(1);
        claw2.setPower(-1);
        clamp = false;
        return;
    }
    public void Close(){
        clamp=true;
        return;
    }
}