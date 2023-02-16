package org.firstinspires.ftc.teamcode.BasicCode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear Opmode")

public class BasicDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor topArm;
    private CRServo servo;
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    private DistanceSensor frontDistance;
    private DistanceSensor rearDistance;

    @Override
    public void runOpMode() {

        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        topArm = hardwareMap.get(DcMotor.class, "topArm");
        servo = hardwareMap.crservo.get("servo");

        sensorColor = hardwareMap.get(ColorSensor.class, "colourSensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colourSensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");


        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max = 0.0;
            double armPower = (gamepad1.right_trigger - gamepad1.left_trigger);

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double speed = 0;

            if (gamepad1.b) {
                speed = 2;
            } else {
                speed = 3.5;
            }


            double leftBackPower = (axial - lateral + yaw) / speed;
            double rightBackPower = (axial + lateral - yaw) / speed;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.right_bumper) {
                servo.setPower(0.5);
            }
            if (gamepad1.left_bumper) {
                servo.setPower(0.0);
            }


            // Send calculated power to wheels
            topArm.setPower(armPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            //Get the normalized colors from the sensor
            //NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Show the elapsed game time and wheel power.
            // telemetry.addData("Servo: ", "%.2f", servo.getPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Back  left Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("Back  right Position", rightBackDrive.getCurrentPosition());

            telemetry.addData("Colour Distance (cm): ", sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Distance (cm): ", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Rear Distance (cm): ", rearDistance.getDistance(DistanceUnit.CM));

            telemetry.addData("Red", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue", sensorColor.blue());

            telemetry.update();
            sleep(20);
        }
    }
}