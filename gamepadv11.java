package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;



/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Gamepad V11", group="Control")
@Disabled
public class gamepadv11 extends OpMode
{
    // Declare OpMode members.
    // initializare componente
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor_stanga_fata = null;
    private DcMotor motor_dreapta_fata = null;
    private DcMotor motor_stanga_spate = null;
    private DcMotor motor_dreapta_spate = null;
    private DistanceSensor sensorRange;
    private DcMotor brat = null;
    private Servo   servo1;
    private Servo   servo2;
    private Servo   servo3;
    private Servo   servo;
    private double  pozitie= 0.5;
    private double  pozitie1 = 0.5;
    private double  pozitie2 = 0.5;
    private double  pozitie3 = 0.5;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor_stanga_fata  = hardwareMap.get(DcMotor.class, "left_drive");
        motor_dreapta_fata = hardwareMap.get(DcMotor.class, "right_drive");
        motor_stanga_spate = hardwareMap.get(DcMotor.class, "left_drive1");
        motor_dreapta_spate = hardwareMap.get(DcMotor.class, "right_drive1");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motor_stanga_fata.setDirection(DcMotor.Direction.FORWARD);
        motor_dreapta_fata.setDirection(DcMotor.Direction.REVERSE);
        motor_stanga_spate.setDirection(DcMotor.Direction.FORWARD);
        motor_dreapta_spate.setDirection(DcMotor.Direction.REVERSE);

        brat = hardwareMap.get(DcMotor.class, "brat");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class,"servo2");
        servo3 = hardwareMap.get(Servo.class,"servo3");
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double stick_stanga_x;
        double stick_stanga_y;
        double stick_dreapta_x;
        double stick_dreapta_y;
        double invarte_in_stanga;
        double invarte_in_dreapta;
        double putere_rotatie = 0.4;
        double optiune=1;
        stick_stanga_x = -gamepad1.left_stick_x ;
        stick_stanga_y = -gamepad1.left_stick_y ;
        stick_dreapta_x = -gamepad1.right_stick_x;
        stick_dreapta_y = -gamepad1.right_stick_y;
        invarte_in_stanga = gamepad1.left_trigger;
        invarte_in_dreapta = gamepad1.right_trigger;
        if(gamepad1.x)
            optiune='m';
        else
        if(gamepad1.b)
            optiune='b' ;
        if(optiune=='m')
        {
            // inceput joystyck dreapta
            //STANGA/DREAPTA
            if(stick_stanga_x!=0)
            {
                motor_stanga_fata.setPower(stick_stanga_x);
                motor_dreapta_fata.setPower(-stick_stanga_x);
                motor_stanga_spate.setPower(-stick_stanga_x);
                motor_dreapta_spate.setPower(stick_stanga_x);
            }
           else
            // FATA/SPATE
            if(stick_stanga_y!=0)
            {
                motor_stanga_fata.setPower(stick_stanga_y);
                motor_dreapta_fata.setPower(stick_stanga_y);
                motor_stanga_spate.setPower(stick_stanga_y);
                motor_dreapta_spate.setPower(stick_stanga_y);
            }
            else

            //sfarsit joystick dreapta
            //inceput joystic stanga
            //daca impingi joystigul in fata && stanga se misca in diagonala fata-stanga
            if(stick_dreapta_y>0 && stick_dreapta_x<0)
            {
                motor_dreapta_fata.setPower(0);
                motor_stanga_spate.setPower(0);
                motor_stanga_fata.setPower(stick_dreapta_x);
                motor_dreapta_spate.setPower(stick_dreapta_x);
            }
            else
                //daca impingi joystigul in fata && dreapta se misca in diagonala fata-dreapta
                if(stick_dreapta_y>0 && stick_dreapta_x>0)
                {
                    motor_dreapta_fata.setPower(stick_dreapta_x);
                    motor_stanga_spate.setPower(stick_dreapta_x);
                }
                else
                    //daca impingi joystigul in spate && stanga se misca in diagonala spate-stanga
                    if(stick_dreapta_y<0 && stick_dreapta_x<0){
                        motor_stanga_fata.setPower(stick_dreapta_x);
                        motor_dreapta_spate.setPower(stick_dreapta_x);}
                    else
                        //daca impingi joystigul in spate && dreapta se misca in diagonala spate-dreapta
                        if(stick_dreapta_y<0 && stick_dreapta_x>0)
                        {
                            motor_dreapta_fata.setPower(stick_dreapta_x);
                            motor_stanga_spate.setPower(stick_dreapta_x);
                        }
                        else
                        {
                            motor_stanga_fata.setPower(0);
                            motor_dreapta_fata.setPower(0);
                            motor_stanga_spate.setPower(0);
                            motor_dreapta_spate.setPower(0);
                            telemetry.addData("Status", "Nu apesi nimic!");
                        }
            //sfarsit joystick stanga
            // rotire stanga
            if(invarte_in_stanga!=0)
            {
                motor_stanga_fata.setPower(-invarte_in_stanga);
                motor_dreapta_fata.setPower(invarte_in_stanga);
                motor_stanga_spate.setPower(-invarte_in_stanga);
                motor_dreapta_spate.setPower(invarte_in_stanga);
            }
            else

            //rotire dreapta
            if(invarte_in_dreapta!=0)
            {
                motor_stanga_fata.setPower(invarte_in_dreapta);
                motor_dreapta_fata.setPower(-invarte_in_dreapta );
                motor_stanga_spate.setPower(invarte_in_dreapta );
                motor_dreapta_spate.setPower(-invarte_in_dreapta );
            }
            else
            {
                motor_stanga_fata.setPower(0);
                motor_dreapta_fata.setPower(0);
                motor_stanga_spate.setPower(0);
                motor_dreapta_spate.setPower(0);
            }
        }
        else
        {
            if (gamepad1.right_stick_y!=0)
            {
                if(gamepad1.right_stick_y>0)
                {
                    if (pozitie1 < 1 && pozitie1 > 0)
                        pozitie1 = pozitie1 + 0.0001;
                }
            else
                if(gamepad1.right_stick_y<0)
                    {
                        if(pozitie1<1 && pozitie1>0.0001)
                            pozitie1= pozitie1 - 0.0001;
                    }
            }
            //brat-servo2-stanga-dreapta
            if(gamepad1.right_stick_x!=0)
            {
                if (pozitie2 < 1 && pozitie2 > 0)
                    pozitie2 = pozitie2 + 0.0001;
                else
                    if (gamepad1.right_stick_x < 0)
                    {
                        if (pozitie2 < 1 && pozitie2 > 0.0001)
                            pozitie2 = pozitie2 - 0.0001;
                    }
            }
            //brat-servo3-prindere
            if(gamepad1.left_stick_x!=0)
            {
                if (pozitie3 < 1 && pozitie3 > 0)
                    pozitie3 = pozitie3 + 0.0001;
                else
                    if (gamepad1.left_stick_x < 0)
                    {
                        if (pozitie2 < 1 && pozitie2 > 0.0001)
                        pozitie3 = pozitie3 - 0.0001;
                    }
            }
            //lift-sus-jos
            if (gamepad1.a)
            {
                brat.setPower(-putere_rotatie);
            }
            else
                if (gamepad1.y)
                {
                brat.setPower(putere_rotatie);
                }
                else
                brat.setPower(0);

            //se afiseaza pozitia servoului
            telemetry.addData("Servo Position", "%5.2f", pozitie1);
            telemetry.addData("Servo Position", "%5.2f", pozitie2);
            telemetry.addData("Servo Position", "%5.2f", pozitie3);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();
            servo1.setPosition(pozitie1);
            servo2.setPosition(pozitie2);
            servo3.setPosition(pozitie3);
        }
        // se afiseaza timpul scurs si puterea
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motoare", "Stanga-Dreapta (%.2f), Fata-Spate (%.2f)", stick_stanga_x, stick_stanga_y);
        telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm",sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }

