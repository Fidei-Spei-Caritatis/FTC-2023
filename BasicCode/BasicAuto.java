package org.firstinspires.ftc.teamcode.BasicCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "PowerPlay Auto")

public class BasicAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    private DistanceSensor frontDistance;
    private DistanceSensor rearDistance;

    @Override
    public void runOpMode() {
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        sensorColor = hardwareMap.get(ColorSensor.class, "colourSensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colourSensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (frontDistance.getDistance(DistanceUnit.CM) > 10.0) {
                leftBackDrive.setPower(0.24); //the robot drifts to the right sightly
                rightBackDrive.setPower(-0.25);
            } else {
                leftBackDrive.setPower(0.0);
                rightBackDrive.setPower(0.0);
            }

            telemetry.addData("Colour Distance (cm): ", sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Distance (cm): ", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Rear Distance (cm): ", rearDistance.getDistance(DistanceUnit.CM));

            telemetry.addData("Red", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue", sensorColor.blue());
            telemetry.update();

        }
    }
}