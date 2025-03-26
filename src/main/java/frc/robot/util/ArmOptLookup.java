package frc.robot.util;

import java.util.HashMap;

import frc.robot.RobotContainer;
import lombok.val;

public class ArmOptLookup {
    

    private HashMap<Double, Boolean> acceptableArmPosition;
    private double[] key_values;
    private double[] values; 
    public ArmOptLookup(double x[], double y[]){

        acceptableArmPosition= new HashMap<>();
        key_values=x;
        values=y;
        

    }

    public ArmOptLookup()
    //linear interpolation
    public double get(int xval){
        double dist=100;
        int lower_i=0;
        int upper_i=0;
        for(int i=0; i<key_values.length-1; i++){
            if (xval-key_values[i]<dist &&  xval-key_values[i] > 0){
                dist=Math.abs(key_values[i] - xval);
                lower_i=i;
                upper_i=i+1;



            }

            

        }

        double slope= values[upper_i] - values[lower_i] / (key_values[upper_i]-key_values[lower_i]);
        double deltaX= xval- key_values[lower_i];
        double deltaY= slope*deltaX;
        return values[lower_i] + deltaY;

        
    }
}
