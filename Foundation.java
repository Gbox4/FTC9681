package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;



public class Foundation{
    Servo ser;
    double pos;

    public Foundation(){
        ser = hardwareMap.servo.get("mrClamp");
        pos=.7;
    }
    public void Use(){
        Update();
        ser.setPosition(pos);
        return;
    }
    private void Update(){
         if (gamepad1.right_bumper && pos>.28){
            pos  -= 0.01;
        }
        else if (gamepad1.left_bumper && pos<.90){
            pos += 0.01;
        }
        return;
    }
    public void Telemetry1(){
        telemetry.addData("Color foundation pos should be", pos);
        telemetry.addData("what it atually is", ser.getPosition());
    }
}