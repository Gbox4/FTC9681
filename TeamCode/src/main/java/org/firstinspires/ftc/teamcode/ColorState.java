package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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



public class ColorState implements State {

    Telemetry telemetry;

    ColorSensor mrSensor;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    boolean checker = false;
    private State NextState;
    String direction;
    String color;
    double totalTime;
    ElapsedTime mRuntime = new ElapsedTime();
    boolean reset = true;
    int time1 = 0;
    boolean done = false;

    public ColorState(ArrayList<DcMotor> motor, ColorSensor colorSensor, String movement, String color1, int time) {
       time1=time;
        color=color1;
        direction = movement;
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);

        mrSensor = colorSensor;


    }

    public void setNextState(State state) {
        NextState = state;

    }

    @Override
    public void start() {


    }

    @Override
    public State update() {
        if (reset) {
            mRuntime.reset();
            reset = false;
        }
        if(color == "alpha") {
            if (mrSensor.alpha() > 10 && mRuntime.milliseconds() < time1) {
                if (direction == "backward") {
                    leftFront.setPower(-.2);
                    rightFront.setPower(-.2);
                    leftBack.setPower(-.2);
                    rightBack.setPower(-.2);
                } else {
                    leftFront.setPower(.2);
                    rightFront.setPower(.2);
                    leftBack.setPower(.2);
                    rightBack.setPower(.2);
                }
                return this;
            } else {
                totalTime = mRuntime.milliseconds();
                rightFront.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);

                done = true;

                return NextState;
            }
        }
       else if(color == "blue") {
            if (mrSensor.blue() > 10 && mRuntime.milliseconds() < time1) {
                if (direction == "backward") {
                    leftFront.setPower(-.2);
                    rightFront.setPower(-.2);
                    leftBack.setPower(-.2);
                    rightBack.setPower(-.2);
                } else {
                    leftFront.setPower(.2);
                    rightFront.setPower(.2);
                    leftBack.setPower(.2);
                    rightBack.setPower(.2);
                }
                return this;
            } else {
                totalTime = mRuntime.milliseconds();
                rightFront.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);

                return NextState;
            }
        }
    else{
            if (mrSensor.red() > 10 && mRuntime.milliseconds() < time1) {
                if (direction == "backward") {
                    leftFront.setPower(-.2);
                    rightFront.setPower(-.2);
                    leftBack.setPower(-.2);
                    rightBack.setPower(-.2);
                } else {
                    leftFront.setPower(.2);
                    rightFront.setPower(.2);
                    leftBack.setPower(.2);
                    rightBack.setPower(.2);
                }
                return this;
            } else {
                totalTime = mRuntime.milliseconds();
                rightFront.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);

                return NextState;
            }

        }
    }
}

