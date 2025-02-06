package frc.robot.util;

import static edu.wpi.first.units.Units.derive;

import java.lang.constant.Constable;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;

public class CommandFactory {

    public static Drive drive;
    public static Arm arm;
    public static Vision vision;
     public static Elevator elevator; 
     public static Intake intake;

    public CommandFactory(Drive drive, Arm arm, Vision vision, Elevator elevator, Intake intake ){

        this.drive=drive;
        this.arm=arm;
        this.vision=vision;
        this.elevator=elevator;
        this.intake=intake;




    }

    public Command elevatorScore(){
        // Run in Parallel w/PathFollow

        return elevator.raise().unless(()-> (!Constants.RobotState.currFieldZone.equals("REEF")));
    }

    public Command intakePiece(){


        return new ConditionalCommand(intake.setIntakeAlgaePosition(), intake.setIntakeCoralPosition(), ()-> (Constants.RobotState.targetGamepiece == Constants.Gamepiece.ALGAE));
    }



    
}
