package org.firstinspires.ftc.teamcode;

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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Linear OpMode", group = "Linear OpMode")

public class KIWITeamCode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightDrive;
    private DcMotor leftDrive;
    private DcMotor backDrive;
    private DcMotor skyLift;
    private CRServo droneLaunch;

    @Override
    public void runOpMode() {

        double vertical;
        double horizontal;
        double pivot;
        double rightMotor;
        double leftMotor;
        double backMotor;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        backDrive = hardwareMap.get(DcMotor.class, "backDrive");
        skyLift = hardwareMap.get(DcMotor.class, "skyLift");
        droneLaunch = hardwareMap.get(CRServo.class, "droneLaunch");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backDrive.setDirection(DcMotor.Direction.REVERSE);
        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        skyLift.setDirection(DcMotor.Direction.REVERSE);
        droneLaunch.setDirection(CRServo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()){

            // run until the end of the match (driver presses STOP) Holonomic Drive Code, Lift & Drone release
            while (opModeIsActive()) {
                vertical = gamepad1.right_stick_y;
                horizontal = gamepad1.right_stick_x;
                pivot = gamepad1.left_stick_x;
                rightMotor = Range.clip((-0.5 * horizontal - (Math.sqrt(3)/2) * vertical)- pivot, -1.0, 1.0);
                leftMotor = Range.clip((-0.5 * horizontal + (Math.sqrt(3)/2) * vertical)- pivot,-1.0, 1.0);
                backMotor = Range.clip((horizontal - pivot), -1.0, 1.0);

                leftDrive.setPower(leftMotor);
                rightDrive.setPower(rightMotor);
                backDrive.setPower(backMotor);

                if (gamepad2.dpad_up) {
                    skyLift.setPower(0.75);
                    } else if (gamepad2.dpad_down) {
                        skyLift.setPower(-0.75);
                    } else {
                        skyLift.setPower(0);
                    }

                if (gamepad2.dpad_up) {
                    droneLaunch.setPower(1);
                    } else if (gamepad2.dpad_down) {
                        droneLaunch.setPower(-1);
                    } else {
                        droneLaunch.setPower(0);
                    }

                // Show the elapsed game time and wheel power.
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Motors", "left (%.2f), right (%.2f), back (%.2f)", leftMotor, rightMotor, backMotor);
                    telemetry.update();
            }
        }
    }
}

