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
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;

public class extendArmState implements State {


    DcMotor extendArm;
    private double Power;
    private State NextState;

    private int Time;
    ElapsedTime mRuntime = new ElapsedTime();


    public extendArmState(int time, double power, DcMotor extend) {
        Time = time;
        extendArm = extend;

        Power = power;


    }

    public void setNextState(State state) {
        NextState = state;

    }

    @Override
    public void start() {
        mRuntime.reset();

    }

    @Override
    public State update() {


        while (mRuntime.milliseconds() < Time) {

            extendArm.setPower(Power);

            return this;

        }
        if (Time <= mRuntime.milliseconds()) {
            extendArm.setPower(0);
            // return NextState;
        }





        return NextState;
    }

}