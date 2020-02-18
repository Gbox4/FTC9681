package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.hardware.Servo;


public class Arm{
    DcMotor raiseArm2;
    DcMotor extendArm;
    public Arm(){
        raiseArm2 = hardwareMap.dcMotor.get("raise arm 2");
        extendArm = hardwareMap.dcMotor.get("extend arm");
        raiseArm2.setDirection(DcMotorSimple.Direction.FORWARD);
        
    }
    public void Use(){
        raiseArm2.setPower(-gamepad2.left_stick_y);
        extendArm.setPower(-gamepad2.right_stick_y);
        return;
    }
}
