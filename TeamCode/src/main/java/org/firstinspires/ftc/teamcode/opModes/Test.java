package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name="teleOpFunnyTest")
public class Test extends CommandOpMode {
    Vision vision;

    @Override
    public void initialize() {
        Vision vision = new Vision(hardwareMap.get(WebcamName.class, "Webcam 1"), telemetry);
    }

    @Override
    public void run(){
        super.run();
        telemetry.update();
    }
}
