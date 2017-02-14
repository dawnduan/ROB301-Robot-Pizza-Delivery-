
import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import java.lang.*;
import java.util.*;

/**
 * ROB301 Group13
 * FinalProj.Java
 * Purpose: Be able to delivery with given pizza fetch spot, through a forests of blocks and send to the assignment house
 *
 * @author M. Duan, X. Zhang, Z. Liu.
 * @version 2.0 12/03/16
 */
public class BlockAvoid {
    /**
     * Sensors:
     *  Gyro - Fetch xy coordinates
     *  Ultrasonic - Block
     *
     */
    public final static int L = -90;
    public final static int R = 90;
    public final static int l = 0;
    public final static int r = 1;
    public final static float DIS_HOURSE = 13;          // sonic detection val
    public final static float H_INCR = 5f;              // houseDetect incr val
    public final static float LANE_LEN = 155;           // *3 TO DO
    public final static float GOAL_X = 153;             // Goal: Exit of Forest of Exit
    public final static float GOAL_Y = 202;
    public final static float PIVOT_BLOCK = 3;          // sonic block detect range
    public final static float ANGLE_YELLOW = 60;
    public final static float ANGLE_GREEN = -60;
    public final static float STEP_SIZE = 3f;
    public final static float LANE_L = 0;
    public final static float LANE_R = 1;
    public final static float LANE_M = 0.5f;
    public final static float[] PIZZA_0 = {93f,18f};    // Pizza Location Coords Initialization
    public final static float[] PIZZA_1 = {213f,18f};
    public final static float FIELD_X = 310f;
    public final static float FIELD_Y = 400f;
    
    public static boolean pizza1 = false;
    public static boolean pizza0 = false;
    public static boolean LHS = false;
    public static boolean RHS = false;
    public static boolean dropPizza;
    public static int houseNum = 0;
    //public static float [][] hLoc = new float [6][2];
    public static float xVal = 0.0f;
    public static float yVal = 0.0f;
    public static EV3UltrasonicSensor sonic;
    public static EV3GyroSensor tilt;
    
    /**
     * Ports Assignment
     * A Arm
     * B Wheel - L
     * C Wheel - r
     * D Ultrasonic
     * S2 Ultra
     * S3 Gyro
     */
    public static void main(String[] args) throws Exception {
    	sonic = new EV3UltrasonicSensor(SensorPort.S2);
        tilt = new EV3GyroSensor(SensorPort.S3);
        int i = 0;
        float tiltsample = 0;
        boolean[] noBlock = new boolean[3];
        float min = 0, curr = 0, lane = 0;                          // min dist towards goal
        float[] temp = new float[2];
        
        float[][] pizzaLoc = {{22, 90, 55, PIZZA_0[0], PIZZA_0[1]}, //float len1, int rotationAmt, float len2
            {16, -90, 55, PIZZA_1[0], PIZZA_1[1]}};                 // l-0; r -1
        /****** User Input Selection *******/
        // Get Pizza
        System.out.println("User Instruction starts: Fetch Pizza Location selects: LeftButton-0, RightButton-1");
        boolean pressed = false;
        int loc = 0;
        while(pressed){
            if(Button.LEFT.isDown()){
                loc = l;
                pizza0 = true;
                pressed = true;
            } else if(Button.RIGHT.isDown()){
                loc = r;
                pizza1 = true;
                pressed = true;
            }
        }
        // Pizza Loc Confirmation
        pressed = false;
        if(pizza1){
            System.out.println("Pizza1 has confirmed, press enter to go to the next step");
        }else{
            System.out.println("Pizza0 has confirmed, press enter to go to the next step");
        }
        while (!Button.ENTER.isDown()){
            i += 1;
        }
        while (Button.ENTER.isDown()){
            i += 1;
        }
        // Lane Selection
        System.out.println("Lane Selection starts: Left lane-0, Middle lane-0.5, Right lane-1");
        while(!pressed){
            if(Button.LEFT.isDown()){
                lane = LANE_L;
                pressed = true;
            } else if (Button.ENTER.isDown()){
                lane = LANE_M;
                pressed = true;
            } else if (Button.RIGHT.isDown()){
                lane = LANE_R;
                pressed = true;
            }
        }
        // Lane Selection Confirmation
        pressed = false;
        System.out.println("Lane "+lane+" has confirmed, where 0-LANE_L, 0.5-LANE_M, 1-LANE_R, press enter to go to the next step");
        while (!Button.ENTER.isDown()){
            i += 1;
        }
        while (Button.ENTER.isDown()){
            i += 1;
        }
        // Side of Lane Selection
        while(!pressed){
            if(Button.LEFT.isDown()){
                LHS = true;
                pressed = true;
            }else if(Button.RIGHT.isDown()){
                RHS = true;
                pressed = true;
            }
        }
        // Lane Side Confirmation
        pressed = false;
        if(LHS){
            System.out.println("LHS has confirmed, press enter to go to the next step");
        }else{
            System.out.println("RHS has confirmed, press enter to go to the next step");
        }
        while (!Button.ENTER.isDown()){
            i += 1;
        }
        while (Button.ENTER.isDown()){
            i += 1;
        }
        // House Selection
        System.out.println("House Selection: Left-House01 (close to the circle), Enter-House02, Right-House03 (close to the end of road)");
        while(!pressed){
            if(Button.LEFT.isDown()){
                houseNum = 1;
                pressed = true;
            }else if(Button.RIGHT.isDown()){
                houseNum = 3;
                pressed = true;
            }else if(Button.ENTER.isDown()){
                houseNum = 2;
                pressed = true;
            }
        }
        // House Selection Confirmation
        pressed = false;
        System.out.println("House 0"+houseNum+" has confirmed. You shall press enter to begin your journey. Good luck!");
        
        // initialization of the robot sensors
        Motor.C.setSpeed(270);
        Motor.B.setSpeed(270);
        /**************get Pizza********************/
        // user input line: if user choose left then loc = l(=0) otherwise
        looseClamp();
        getPizza(pizzaLoc[loc][0], pizzaLoc[loc][1], pizzaLoc[loc][2]);
        xVal = pizzaLoc[loc][3];
        yVal = pizzaLoc[loc][4];                                // Sarting Loc Set-Up
        bodyRotate(90);
        /***********Bloc Avoid - AStar*************/
        while((GOAL_X-xVal != 0)&& GOAL_Y-yVal != 0){
            noBlock = checkBlock(PIVOT_BLOCK);
            System.out.println("Results in the three directions front: "+noBlock[0]+"; Left: "+noBlock[1]+"; Right: "+noBlock[2]+" ");
            min = temp[0];
            for(i = 0; i<3; i++){
                
                if(noBlock[i] != true){
                    // cost calcul
//                    curr = disttoGoal(xVal,yVal);
//                    
//                    temp[0] = (min <= curr) ? min : curr;
//                    temp[1] = (min <= curr) ? temp[1] : i;
                	min = i;
                }
            }
            //int tem = (int)temp[1];
//            switch(tem){
//                case 0: break;                      // straight
//                case 1: bodyRotate(90);
//                		break;// left
//                case 2: bodyRotate(-90);  
//                		break;// right
//            }
            if(min == true){
            	bodyRotate(90);
            }else if(noBlock[1] == true){
            	bodyRotate(-180);
            } 
//            System.out.println("...\n"+tem+"\n...");
//            if(noBlock[0] == ){
//            	bodyRotate(90);
//            }else if(noBlock[1] == 2){
//            	bodyRotate(-90);
//            }
            
            tiltsample = gyro();
            goStraight(tiltsample, -STEP_SIZE);
        }
        
        
        /********************************	Deliver Pizza given Lane 	**********************/
        // would it work for Y/G Lane? YES: distance counted by the H_INCR, indep of orientation
        //				i = 0;
        //				while(i<=2){
        //					tiltsample = gyro();
        //					houseDetect(tiltsample,DIS_HOURSE,i,L);
        //                    i += 1;
        //				}
        //				bodyRotate(180);
        //
        //				while(i<=5){
        //                    houseDetect(tiltsample,DIS_HOURSE,i,R);
        //                    i += 1;
        //				}
        //float lane1;
        tiltsample = gyro();
        bodyFacingVertical(tiltsample);             // Resume Facing north, 
        

        if(lane == LANE_L){
        	 bodyRotate(ANGLE_YELLOW);
        } else if (lane == LANE_R){
        	bodyRotate(ANGLE_GREEN);
        }

        // Adjust Sonic Facing
        if(LHS){
            sonicRotate(L);
        }else{
            sonicRotate(R);
        }
        
        //int houseDetected = 0;
        while(i < houseNum){
            tiltsample = gyro();
            //boolean detected = false;
            houseDetect(tiltsample,DIS_HOURSE);
            //if (detected){
            //houseDetected += 1; //
            //detected = false;
            //}
            // Havn't found in it from the the left side
            //            if(houseDetected = 3) {
            //                if(dir != l){
            //                    sonicRotate(90);
            //                    dir = r;
            //                    houseDetected += 1;
            //                }
            //            }
            i++;
        }
        //				// go to circle
        //				float disttoCir = LANE_LEN - hLoc[5][0];
        //				i = 0;
        //				while (disttoCir - H_INCR*i >= 0) {
        //					goStraight(tiltsample,H_INCR);
        //					i += 1;
        //				}
        //				bodyRotate(180);
        // UI TODO user_input
        //int houseNum = user_Input;
        looseClamp();
        
        
        
        
        
        // Displays
        //        LCD.clear();
        //            //System.out.println(redsample[0]);
        //        System.out.println(tiltsample[0] + " " + ratesample[0]);
        //            System.out.println(sonicsample[0]*100);
        
        //        }
        //        color.close();
        //        sonic.close();
        /***************** GO Back*******************/
        tiltsample = gyro();
        // facing west
        float diff = 180 - tiltsample;
        bodyRotate(diff);
        tiltsample = gyro();
        goStraight(tiltsample, FIELD_X - xVal);
        bodyRotate(90); //facing south
        tiltsample = gyro();
        goStraight(tiltsample, FIELD_Y - yVal);
        
        
    }
    
    /**
     * The following two methods are used to initialize the sensors. 
     * It offers an convenient way to utilize the sensor
     */
    public static float ultrasonic(){
        int sampleSonicSize = sonic.sampleSize();
        float[] sonicsample = new float[sampleSonicSize];
        sonic.fetchSample(sonicsample, 0);
        return sonicsample[0];
    }
    
    public static float gyro(){
        int sampleGyroSize = tilt.sampleSize();
        float[] tiltsample = new float[sampleGyroSize];
        //float[] ratesample = new float[sampleGyroSize];
        tilt.getAngleMode().fetchSample(tiltsample, 0);
        //tilt.getRateMode().fetchSample(ratesample, 0);
        return tiltsample[0];
    }
    
    /**
     * This method is used to initialize the base motors speeds.
     * @param v_c This is the first parameter to setSpeed method
     * @param v_b This is the second parameter to setSpeed method
     */
    public static void setSpeed(int v_c, int v_b){
        Motor.C.setSpeed(v_c);
        Motor.B.setSpeed(v_b);
    }
    
    /**
     * This method is used to drive the robot go straight. This is
     * One way is to directly send rotation amount to motors
     * Alternative: rotate until the readings from gyrometer is equal to the desired one
     * After the rotation, it updates the x,y coordniates value based on odemetry
     * @param amt This is the paramter to bodyRotate method
     */
    public static void goStraight(float heading, float incr){
        // 1cm
        incr *= 20.83f;
        //float l = 0.022;
        Motor.B.rotate((int)(incr), true);
        Motor.C.rotate((int)(incr));
        // Record of x, y pos
        xVal += (float)Math.cos(heading)*incr;
        yVal += (float)Math.sin(heading)*incr;
    }
    
    /**
     * This method is used to rotate the robot body via motor B and C. This is
     * One way is to directly send rotation amount to motors
     * Alternative: rotate until the readings from gyrometer is equal to the desired one
     * @param amt This is the paramter to bodyRotate method
     */
    public static void bodyRotate(float amt){
        //  1 degree
    	double ratio = 818.18/360;
    	amt = (float) (amt*ratio);
    	Motor.B.rotate((int)amt, true);
        Motor.C.rotate(-(int)amt);
        /*
        float tiltsample = gyro();
        while(tiltsample != amt){
            Motor.B.rotate((int)10, true);
            Motor.C.rotate(-(int)10);
            tiltsample = gyro();
        }
        */
    }
    
    /**
     * This method is used to rotate the ultra sensor via motor D.
     * @param amt This is the paramter to sonicRotate method
     */
    public static void sonicRotate(int amt){
        Motor.D.rotate(amt);
    }
    
    /**
     * This method is used to detect obstocles/house.
     * @param sonicSample This is the first paramter to sonicTest method, indicating the readings from ultra sensor
     * @param num  This is the second parameter to sonicSample method
     * @return boolean This returns the result: whether or not sth. is detected. True: detected, False: not detected
     */
    public static boolean sonicTest(float sonicSample ,float num){
        boolean test = sonicSample < num;
        return test;
    }
    
    /**
     * This method is used to check if there's obstacles in the three directions, i.e. left, front and right
     * It's a crutial step to A* star search algorithm.
     * @param sonicSample This is the first paramter to sonicTest method, indicating the readings from ultra sensor
     * @return boolean[] This returns a array contains the results from the 3 directions
     */
    public static boolean[] checkBlock(float PIVOT_BLOCK){
        boolean[] noBlock = new boolean[3];
        float sonicSample = ultrasonic();
        // Straight pos_s
        noBlock[0] = sonicTest(sonicSample, PIVOT_BLOCK);
        sonicRotate(-90); //to Left
        // Left pos_l
        sonicSample = ultrasonic();
        noBlock[1] = sonicTest(sonicSample, PIVOT_BLOCK);
        sonicRotate(180); //to Right
        //Right pos_r
        sonicSample = ultrasonic();
        sonicRotate(-90);// back: Align with body
        noBlock[2] = sonicTest(sonicSample, PIVOT_BLOCK); // left
        
        return noBlock;
    }
    
    /**
     * This method is used to grab the pizza. The path is predefined
     * @param len1 This is the first parameter to getPizza method
     * @param pizzaLoc  This is the second parameter to getPizza method
     * @param len2 This is the third parameter to getPizza method
     */
    public static void getPizza(float len1, float pizzaLoc, float len2 ){
        float tiltsample = gyro();
        goStraight(tiltsample, len1);
        bodyRotate(pizzaLoc);
        tiltsample = gyro();
        goStraight(tiltsample, len2);
        tightenClamp();
    }
    
    /**
     * This method is used to calculate the next distance to the goal.
     * @param xVal This is the first paramter to disttoGoal method
     * @param yVal  This is the second parameter to disttoGoal method
     * @return float This returns the distance between (xVal, yVal) and (GOAL_X, GOAL_Y).
     */
    public static float disttoGoal(float xVal, float yVal){
        double dis = Math.sqrt(xVal*xVal + yVal*yVal);
        return (float)dis;
    }
    
    /**
     * This method is used to report the house detection.
     * @param xVal This is the first paramter to disttoGoal method
     * @param yVal  This is the second parameter to disttoGoal method
     */
    public static void houseDetect(float heading, float DIS_HOURSE){
        // sonic is facing house
        //int i = 0;
        //boolean detected = false;
        float sonicSample = ultrasonic();
        while(sonicSample > DIS_HOURSE){
            goStraight(heading,H_INCR);
            //i += 1;
        }
    }
    
    /**
     * This method is used to tighten the clamp, i.e. close.
     * It utilizes motor A to realize the action, close.
     * It's also used when grabbing the pizza
     */
    public static boolean tightenClamp(){
        if(dropPizza){
            return false;
        }
        Motor.A.rotate(-95);
        dropPizza = true;
        return true;
    }
    
    /**
     * This method is used to loose the clamp, i.e. open.
     * It utilizes motor A to realize the action, open.
     * It's also used when the delivery completes, the pizza drops
     */
    public static boolean looseClamp(){
        if(!dropPizza){
            return false;
        }
        Motor.A.rotate(95);
        dropPizza = false;
        return true;
    }
    
    public static void bodyFacingVertical(float tiltsample){
        // Adjust body facing vertical
        float diff = tiltsample - 90;
        bodyRotate(diff);
        
    }
    
}
