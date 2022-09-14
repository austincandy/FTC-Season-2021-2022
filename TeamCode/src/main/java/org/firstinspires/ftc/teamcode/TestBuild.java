/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Linear Opmode")
//@Disabled
public class TestBuild extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftRear, rightRear, liftMotor, spinner, spintake;
    private CRServo horiz, vert, tape;
    private double v1, v2, v3, v4;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        spintake = hardwareMap.get(DcMotor.class, "spintake");
        horiz = hardwareMap.get(CRServo.class, "horiz");
        vert = hardwareMap.get(CRServo.class, "vert");
        tape = hardwareMap.get(CRServo.class, "tape");


        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

//
//            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
//            double rightX = -gamepad1.right_stick_x;
//            double v1 = -r * Math.sin(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 4) + rightX;
//            double v2 = r * Math.sin(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4) + rightX;
//            double v3 = -r * Math.sin(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4) + rightX;
//            double v4 = r * Math.sin(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 4) + rightX;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double v1 = (y + x + rx) / denominator;
//            double v2 = (y - x + rx) / denominator;
//            double v3  = (y - x - rx) / denominator;
//            double v4 = (y + x - rx) / denominator;
              double drive = -gamepad1.left_stick_y;
              double turn  =  gamepad1.right_stick_x;
              v1    = Range.clip(drive + turn, -1.0, 1.0) ;
              v2   = Range.clip(drive - turn, -1.0, 1.0) ;
              v3    = Range.clip(drive + turn, -1.0, 1.0) ;
              v4   = Range.clip(drive - turn, -1.0, 1.0) ;

            if (gamepad1.left_stick_x < -0.7){
                v1 = (0 + (gamepad1.left_stick_x * 1.1) + 0) / Math.max(Math.abs(0) + Math.abs(-1.1) + Math.abs(0), 1);
                v2 = (0 - (gamepad1.left_stick_x * 1.1) + 0) / Math.max(Math.abs(0) + Math.abs(-1.1) + Math.abs(0), 1);
                v3 = (0 - (gamepad1.left_stick_x * 1.1) - 0) / Math.max(Math.abs(0) + Math.abs(-1.1) + Math.abs(0), 1);
                v4 = (0 + (gamepad1.left_stick_x * 1.1) - 0) / Math.max(Math.abs(0) + Math.abs(-1.1) + Math.abs(0), 1);
            }
            if (gamepad1.left_stick_x > 0.7){
                v1 = (0 + (gamepad1.left_stick_x * 1.1) + 0) / Math.max(Math.abs(0) + Math.abs(-1.1) + Math.abs(0), 1);
                v2 = (0 - (gamepad1.left_stick_x * 1.1) + 0) / Math.max(Math.abs(0) + Math.abs(-1.1) + Math.abs(0), 1);
                v3 = (0 - (gamepad1.left_stick_x * 1.1) - 0) / Math.max(Math.abs(0) + Math.abs(-1.1) + Math.abs(0), 1);
                v4 = (0 + (gamepad1.left_stick_x * 1.1) - 0) / Math.max(Math.abs(0) + Math.abs(-1.1) + Math.abs(0), 1);
            }
//            if (Math.abs(gamepad1.right_stick_x) > 0) {
//                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                v1 = -gamepad1.right_stick_x;
//                v2 = -gamepad1.right_stick_x;
//                v3 = gamepad1.right_stick_x;
//                v4 = gamepad1.right_stick_x;
//            } else {
//                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                float yValue = gamepad1.left_stick_y * -1;
//                float xValue = gamepad1.left_stick_x * -1;
//
//                float leftPower =  yValue - xValue;
//                float rightPower = yValue + xValue;
//
//                v1 = -(Range.clip(leftPower, -1.0, 1.0));
//                v2 = Range.clip(rightPower, -1.0, 1.0);
//                v3 = -(Range.clip(leftPower, -1.0, 1.0));
//                v4 = Range.clip(rightPower, -1.0, 1.0);
//            }
            double liftPower = 0, spinnerPower = 0, spintakePower = 0, hPower = 0, vPower = 0, tPower = 0;

            if (gamepad1.y)
                spinnerPower = -0.6;

            if (gamepad1.b)
                spinnerPower = 0.6;

            if(gamepad1.x)
                tPower = 1;

            if(gamepad1.a)
                tPower = -1;

            if(gamepad1.dpad_up)
                vPower = 1;

            if(gamepad1.dpad_down)
                vPower = -1;

            if(gamepad1.dpad_left)
                hPower = 1;

            if(gamepad1.dpad_right)
                hPower = -1;

            if (gamepad1.right_trigger > 0.1) {
                spintakePower = -gamepad1.right_trigger;
            }

            if (gamepad1.left_trigger > 0.1) {
                spintakePower = gamepad1.left_trigger;
            }

            if(gamepad1.right_bumper) {
                liftPower = 1;
            }

            if(gamepad1.left_bumper) {
                liftPower = -1;
            }

            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(-v3);
            rightRear.setPower(-v4);
            liftMotor.setPower(liftPower);
            spinner.setPower(spinnerPower);
            spintake.setPower(spintakePower);
            horiz.setPower(hPower);
            vert.setPower(vPower);
            tape.setPower(tPower);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", liftMotor.getCurrentPosition(), rightFront.getCurrentPosition(), rightRear.getCurrentPosition(), leftRear.getCurrentPosition());
            telemetry.addData("coords", "x (%.2f), y (%.2f)", gamepad1.left_stick_x, -gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}