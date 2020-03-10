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
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: Scan Servo", group = "Concept")
@Disabled
public class testbrat extends LinearOpMode {

         // period of each cycle
       // Minimum rotational position

    // Define class members
        Servo   servo;
Servo servo1;
    Servo   servo2;
    Servo   servo3;
    double  position = 0.5;
    double position1 = 0.5;
    double  position2 = 0.5;
    double  position3 = 0.5;
    private DcMotor brat = null;



    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = hardwareMap.get(Servo.class, "servo1");
        servo1= hardwareMap.get(Servo.class,"servo");
        servo2 = hardwareMap.get(Servo.class,"servo2");
        servo3 = hardwareMap.get(Servo.class,"servo3");
        brat = hardwareMap.get(DcMotor.class, "brat");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){
            boolean a;
            boolean y;
            double power_rotate = 0.4;
            a = gamepad1.a;
            y = gamepad1.y;
            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.right_stick_y!=0 && position<1.00000001 &&position>-0.00000001) {
                // Keep stepping up until we hit the max value.
                position += gamepad1.right_stick_y/1000;
                position1 -= gamepad1.right_stick_y/1000;
            }


            if(gamepad1.right_stick_x!=0 && position2<1 &&position2>0)
            {
                position2+= gamepad1.right_stick_x/1000;
            }


            if(gamepad1.left_stick_x!=0 && position3<1 &&position3>0)
            {
                position3+=gamepad1.left_stick_x/1000;
            }

            if (a) {
                brat.setPower(-power_rotate);
            } else if (y) {
                brat.setPower(power_rotate);
            } else
                brat.setPower(0);

            // Display the current value
            telemetry.addData("Servo Position1", "%5.2f", position);
            telemetry.addData("Servo Position2", "%5.2f", position2);
            telemetry.addData("Servo Position3", "%5.2f", position3);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            servo2.setPosition(position2);
            servo3.setPosition(position3);

        }


        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
