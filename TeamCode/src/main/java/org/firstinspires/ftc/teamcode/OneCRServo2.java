package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;



public class OneCRServo2 implements State {

    Telemetry telemetry;

    CRServo servo1, servo2;
    private double Power;
    private double Power2;
    private String Movement;
    private State NextState;

    private int totalTime;
    ElapsedTime mRuntime = new ElapsedTime();
    boolean reset = true;


    public OneCRServo2(int time, double power, CRServo drag) {
        totalTime = time;
        servo1 = drag;


        Power = power;



    }

    public void setNextState(State state) {
        NextState = state;

    }

    @Override
    public void start() {


    }

    @Override
    public State update() {

        if (reset){
            mRuntime.reset();
            reset = false;
        }


        while (mRuntime.milliseconds() < totalTime) {


            servo1.setPower(Power);

            return this;

        }



        // return NextState;





        return NextState;
    }

}

