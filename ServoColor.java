package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;



public class ServoColor{
    Servo ser;
    double pos;

    public ServoColor(){
        ser = hardwareMap.servo.get("mrServo");
        pos=.5;
    }
    public void Use(){
        Update();
        ser.setPosition(pos);
        return;
    }
    private void Update(){
         if (gamepad1.x/* && draga > -.5*/){
            pos  -= 0.01;
        }
        else if (gamepad1.y){
            pos += 0.01;
        }
        return;
    }
    public void Telemetry1(){
        telemetry.addData("Color servo pos should be", pos);
        telemetry.addData("what it atually is", ser.getPosition());
    }
}