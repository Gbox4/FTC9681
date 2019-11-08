package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;

public class pickUpState implements State {
    int time;
    Servo pickUp1, pickUp2;
    ElapsedTime Runtime = new ElapsedTime();
    private double Pos;
    private double Pos2;


    private State NextState;



    //this state is for setting 2 servos to positions. It will be used for both picking stuff up, and dragging the foundation

    public pickUpState (double pos, double pos2,ArrayList<Servo> servos, int Time) {
        time = Time;
        pickUp1 = servos.get(0);
        pickUp2 = servos.get(1);
       // wanted positions of the servos
        Pos = pos;
        Pos2 = pos2;



    }

    public void setNextState(State state) {
        NextState = state;

    }

    @Override
    public void start() {

    }

    @Override
    public State update() {

       pickUp1.setPosition(Pos);

       pickUp2.setPosition(Pos2);


       // pickUp1.setPosition(pickUp1.get)

    // pickUp1.setPosition(Pos);
   //  pickUp2.setPosition(Pos2);


        return NextState;
    }
}








//        public void wait(int time) {
//        try {
//            Thread.sleep(time * 100);//milliseconds
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

