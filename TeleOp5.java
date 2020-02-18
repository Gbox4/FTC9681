package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name ="TeleOp5", group = "TeleOP")
public class TeleOp5 extends OpMode {
  
    ColorSensor mrSensor;


    public void init() {
        //hardware map is for phone
        // i could Probably use one class for all the normal servos, oh well
        Chasse chasse;
        Arm arm;
        ServoColor servoColor;
        Capstone capstone;
        Foundation foundation;
        Claws grabs;
        Wrist wrist;
       
        mrSensor = hardwareMap.colorSensor.get("mrSensor");
    }


    public void loop() {
        chasse.Drive();
        arm.Use();
        capstone.Use();
        capstone.Telemetry1();
        servoColor.Use();
        servoColor.Telemetry1();
        foundation.Use();
        telemetry.addData("mrSensor values", mrSensor.alpha());
        grabs.Use();
        if(gamepad2.x){
            grabs.Close();
        }
        else if(gamepad2.y){
            grabs.Open();
        }
        wrist.Use();
        telemetry.update();



    }


}