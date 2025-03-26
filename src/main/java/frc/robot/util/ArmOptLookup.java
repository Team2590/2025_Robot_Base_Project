package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import lombok.val;

public class ArmOptLookup {
    //Add optimal ArmPosition given an Elevator Position so we can synchronize movement optimally

    private HashMap<Double, Boolean> acceptableArmPosition;
    private ArrayList<Double> key_values;
    private ArrayList<Double> values; 
    public ArmOptLookup(ArrayList<Double> x, ArrayList<Double> y){

        acceptableArmPosition= new HashMap<>();
        key_values=x;
        values=y;
        

    }

   
    //linear interpolation
    public double get(){
        double xval=RobotContainer.getElevator().getRotationCount();
        

        double dist=100;
        int lower_i=0;
        int upper_i=0;
        for(int i=0; i<key_values.size()-1; i++){
            if (xval-key_values.get(i)<dist &&  xval-key_values.get(i) > 0){
                dist=Math.abs(key_values.get(i) - xval);
                lower_i=i;
                upper_i=i+1;



            }

            

        }

        double slope= values.get(upper_i) - values.get(lower_i) / (key_values.get(upper_i)-key_values.get(lower_i));
        double deltaX= xval- key_values.get(lower_i);
        double deltaY= (xval < Constants.ArmConstantsLeonidas.OPT_TABLE_CLAMP) ? 0:  slope*deltaX;
        return values.get(lower_i) + deltaY;

        
    }

    public Command populate(Trigger t){
        
        ArrayList<Double> elevatorPos= new ArrayList<>();
        ArrayList<Double> armPos =new ArrayList<>();
        return Commands.run(()-> {


            key_values.add(RobotContainer.getElevator().getRotationCount());
            values.add(RobotContainer.getArm().getSetpoint());

        });

        
    }
}
