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



public class CRServoState implements State {

    Telemetry telemetry;

    CRServo servo1, servo2;
    private double Power;
    private double Power2;
    private String Movement;
    private State NextState;

    private int totalTime;
    ElapsedTime mRuntime = new ElapsedTime();
    boolean reset = true;


    public CRServoState(int time, double power, double power2, ArrayList<CRServo> CRServos) {
        totalTime = time;
        servo1 = CRServos.get(0);
        servo2 = CRServos.get(1);

        Power = power;
        Power2 = power2;


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
            servo2.setPower(Power2);
            return this;

        }

        servo1.setPower(0);
        servo2.setPower(0);
        // return NextState;





        return NextState;
    }

}

