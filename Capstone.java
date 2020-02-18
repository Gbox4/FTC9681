package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;



public class Capstone{
    Servo cap;
    double pos;

    public Capstone(){
        cap = hardwareMap.servo.get("drag front");
        pos=-.3;
    }
    public void Use(){
        Update();
        cap.setPosition(pos);
        return;
    }
    private void Update(){
         if (gamepad1./* && draga > -.5*/){
            pos  -= 0.01;
        }
        else if (gamepad1.a && pos< .4){
            pos += 0.01;
        }
        return;
    }
    public void Telemetry1(){
        telemetry.addData("Drag servo pos should be", pos);
        telemetry.addData("what it atually is", cap.getPosition());
    }
}