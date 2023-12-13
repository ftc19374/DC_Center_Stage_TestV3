package org.firstinspires.ftc.teamcode;
/*Tele-Op Program for the Center Stage Competition. Functions include:
Robot Movement (Forward, Backward, Strafing, Turning)
Intake mechanism (conveyor)
Lift Control
Box (open/close)
Drone Launch
Hanging mechanism (Not Implemented at current time)
 */
import android.media.MediaCodecInfo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="DC_Center_Stage_TestV3")

public class DC_Center_Stage_TestV3 extends LinearOpMode{
    DcMotor Front_Right_Wheel = null; //Dc Motor Variable for Front Right Wheel
    DcMotor Front_Left_Wheel  = null; //Dc Motor Variable for Front Left Wheel
    DcMotor Back_Right_Wheel  = null; //Dc Motor Variable for Back Right Wheel
    DcMotor Back_Left_Wheel   = null; //Dc Motor Variable for Back Left Wheel
    DcMotor Intake_Motor    = null; //Dc Motor Variable for Intake Motor
    DcMotor Lift_Motor      = null; //Dc Motor Variable for Lift Motor
    Servo   Box_Servo       = null; //180 degree Servo Variable for Box Mechanism
    DcMotor Drone_Motor     = null; //Dc Motor Variable for Drone Launch Mechanism
    CRServo   Drone_Servo   = null; //Continuous Servo Variable for Drone Launch Mechanism
    DcMotor Hang_Motor      = null; //Dc Motor Variable for Hanging Mechanism
    double IPower = 0; //Intake power variable
    float LPower = 0;  //Lift power variable
    float BPosition = 0;  //Box position variable
    float DroneS = 0; //Servo Power Variable for Drone
    float DroneM = 0; //Motor Power Variable for Drone
    float HPower = 0; //Hanging mechanism Power Variable
    double Cspeed = 1; //chassis speed variable
    @Override
    public void runOpMode() {

        //Config names below
        Front_Right_Wheel = hardwareMap.get(DcMotor.class, "Fr_motor");
        Front_Right_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Front_Left_Wheel  = hardwareMap.get(DcMotor.class, "Fl_motor");
        Back_Right_Wheel  = hardwareMap.get(DcMotor.class, "Br_motor");
        Back_Right_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Left_Wheel   = hardwareMap.get(DcMotor.class, "Bl_motor");
        Intake_Motor    = hardwareMap.get(DcMotor.class, "Intake_Motor_1");
        Lift_Motor      = hardwareMap.get(DcMotor.class, "Lift_1");
        Box_Servo       = hardwareMap.get(Servo.class, "Box_Servo");
        Drone_Motor     = hardwareMap.get(DcMotor.class, "Drone_Motor");
        Drone_Servo     = hardwareMap.get(CRServo.class, "Drone_Servo");
        Hang_Motor      = hardwareMap.get(DcMotor.class, "Hang_Motor");
        Lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Pos: ", Lift_Motor.getCurrentPosition());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {

            //Equations for Robot wheels for the left & right joysticks on gamepad 1
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            Front_Right_Wheel.setPower((y + x + rx)*Cspeed);
            Front_Left_Wheel.setPower((y - x - rx)*Cspeed);
            Back_Left_Wheel.setPower((y + x -  rx)*Cspeed);
            Back_Right_Wheel.setPower((y - x + rx)*Cspeed);

            if (gamepad1.dpad_up) {
                Cspeed = 1;
            }
            else if (gamepad1.dpad_down) {
                Cspeed = 0.5;
            }
            //5th motor controls intake via the left and right bumpers
            // right bumper on gamepad 1 adds power to intake motor to draw in pixels
            if (gamepad1.right_bumper) {
                IPower = 0.85;  //Starts intake Mechanism; pulls pixels in
                Intake_Motor.setPower(IPower);
            }
            //left bumper on gamepad 1 pushes pixels out to prevent control of too many pixels at once
            else if (gamepad1.left_bumper) {
                IPower = -1;  //Reverses intake Mechanism; pushes pixels out
                Intake_Motor.setPower(IPower);
            }
            if (gamepad1.x) {
                //x on the gamepad 1 changes the power in the intake motor to 0, stopping the intake motor
                IPower = 0;  //stops intake
                Intake_Motor.setPower(IPower);
            }
            //raise and lower the lift to be able to reach higher on the backdrop
            if (gamepad2.left_trigger >= 0.5 /*&& Lift_Motor.getCurrentPosition() < 0*/) {
                //left trigger on gamepad 2 moves the lift down & includes position statements to prevent lift from going too far down
                LPower = 1;
                Lift_Motor.setTargetPosition(0);
                Lift_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift_Motor.setPower(LPower);
                sleep(2000);
                Lift_Motor.setPower(0);
                Lift_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad2.right_trigger >= 0.5 /*&& Lift_Motor.getCurrentPosition() >= -2950*/) {
                //right trigger on gamepad 2 moves lift up & includes position statements to prevent lift from going too far up
                LPower = -1;
                Lift_Motor.setTargetPosition(-2950);
                Lift_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift_Motor.setPower(LPower);
                sleep(2000);
                Lift_Motor.setPower(0);
                Lift_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            //Open and close the box function to drop pixels into backdrop
            if (gamepad2.left_bumper) {
                //left bumper on gamepad 2 opens the box
                BPosition = 1f;
                Box_Servo.setPosition(0.5);

            }
            if (gamepad2.right_bumper) {
                //right bumper on gamepad 2 closes the box
                BPosition = 1;
                Box_Servo.setPosition(1);

            }
            //Launch the drone from the robot using a servo and a motor
            if (gamepad2.dpad_right) {
                //launches drone when pressed using the right dpad
                DroneM = 0.5f;
                Drone_Motor.setPower(DroneM);
                sleep(300);
                DroneS = 1;
                Drone_Servo.setPower(DroneS);
                sleep(300);
                DroneM = 0;
                DroneS = 0;
                Drone_Motor.setPower(DroneM);
                Drone_Servo.setPower(DroneS);
            }
            if (gamepad2.dpad_up) {
                //Raises the hanging lift using up on the dpad
                HPower = 0.5f; //Check Speed
                Hang_Motor.setPower(HPower);
            }
            else if (gamepad2.dpad_down) {
                //Lowers the Hanging lift using down on the dpad
                HPower = -0.5f; //Check Speed
                Hang_Motor.setPower(HPower);
            }





            telemetry.addData("Pos: ", Lift_Motor.getCurrentPosition()); //adds telemetry data to the Driver Station
            telemetry.update();
        }
    }
}
