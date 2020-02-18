package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;



public class Wrist{
    Servo ser;
    double pos;

    public Wrist(){
        ser = hardwareMap.servo.get("drag front");
        pos=0;
    }
    public void Use(){
        Update();
        ser.setPosition(pos);
        return;
    }
    private void Update(){
         if (gamepad2.right_bumper && pos>-0.7){
            pos  -= 0.01;
        }
        else if (gamepad2.left_bumper && pos<0.7){
            pos += 0.01;
        }
        return;
    }
    public void Telemetry1(){
        telemetry.addData("Wrist should be", pos);
        telemetry.addData("what it atually is", ser.getPosition());
    }
}