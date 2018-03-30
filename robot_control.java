package csl_final_project;

import java.util.*;
import java.util.HashMap;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;

import java.util.ArrayList;


class node
{
    int x;
    int y;
    int value;
    node parent;
    int visited = 0;
    public node(int x1, int y1,int val)
    {
       x = x1;
       y = y1; 
       value = val;
    }
}


public class robot_control {
	public static EV3GyroSensor tilt = new EV3GyroSensor(SensorPort.S4);
	public static int sampleSize = tilt.sampleSize();
	public static float[] tiltsample = new float[sampleSize];
	
    public static ArrayList path(node[][] map,node start,node end)
    {
        ArrayList<node> path = new ArrayList<node>();
        ArrayList<node> dir = new ArrayList<node>();
        ArrayList<node> q = new ArrayList<node>();
        ArrayList<node> visited = new ArrayList<node>();
        
        
        //direction list
        node temp_node = new node(1,0,0);
        dir.add(temp_node);
        temp_node = new node(-1,0,0);
        dir.add(temp_node);
        temp_node = new node(0,1,0);
        dir.add(temp_node);
        temp_node = new node(0,-1,0);
        dir.add(temp_node);
        
        start.parent = null;
        q.add(map[start.y][start.x]);
        node current = null;
        while(!q.isEmpty())
        {
            current = q.get(0);
            q.remove(0);
            if (current.x == end.x && current.y == end.y)
            {
                break;
            }
            
            for (int i = 0; i < dir.size(); i++) {
    			node direction = dir.get(i);
            	
                if ((current.y + direction.y >= 0 && current.y + direction.y <= 8) && ((current.x + direction.x >= 0 && current.x + direction.x <= 8)) && map[current.y + direction.y][current.x + direction.x].value != 1)
                {
                	//System.out.print(current.y + direction.y);
                	//System.out.println(current.x + direction.x);
                    node child = map[current.y + direction.y][current.x + direction.x];
                    if(child.visited == 0)
                    {
                    	child.parent = current;
                        q.add(child);
                    }
                 
                }
                
            }
            current.visited = 1;
            visited.add(current);
        }
       
        
        
        while(current.x != start.x || current.y != start.y)
        {
            path.add(0,current);
            current = current.parent;
        }
        path.add(0,current);
       
        return path;
    }
	
	public static void gotoAngle(float angle) throws InterruptedException{
		// set speed
		Motor.B.setSpeed(50); // left
		Motor.C.setSpeed(50); // right
		
		/*
		sampleSize = tilt.sampleSize();
		tiltsample = new float[sampleSize];
		tilt.getAngleMode().fetchSample(tiltsample, 0);
		*/
		
		
		if (tiltsample[0] > angle ) {
			Motor.B.forward();
			Motor.C.backward();
			while (tiltsample[0] > angle) {
				//sampleSize = tilt.sampleSize();
				//tiltsample = new float[sampleSize];
				tilt.getAngleMode().fetchSample(tiltsample, 0);
				System.out.println(tiltsample[0]);
			}
		}
		else {
			Motor.B.backward();
			Motor.C.forward();
			while (tiltsample[0] < angle) {
				//sampleSize = tilt.sampleSize();
				//tiltsample = new float[sampleSize];
				tilt.getAngleMode().fetchSample(tiltsample, 0);
				System.out.println(tiltsample[0]);
			}
		}
		
		Motor.B.stop(true);
		Motor.C.stop(true);
	}
	
	public static void firstAngle(float angle) throws InterruptedException{
		// set speed
		Motor.B.setSpeed(50); // left
		Motor.C.setSpeed(50); // right
		
		tilt.reset();
		int wait = 0;
		while(wait<1000)
		{
			wait = wait + 1;
		}
		
		//sampleSize = tilt.sampleSize();
		//tiltsample = new float[sampleSize];
		tilt.getAngleMode().fetchSample(tiltsample, 0);
		
		
    	//System.out.println(tiltsample[0]);
    	
		
		if (tiltsample[0] > angle ) {
			Motor.B.forward();
			Motor.C.backward();
			while (tiltsample[0] > angle) {
				//sampleSize = tilt.sampleSize();
				//tiltsample = new float[sampleSize];
				tilt.getAngleMode().fetchSample(tiltsample, 0);
				System.out.println(tiltsample[0]);
			}
		}
		else {
			Motor.B.backward();
			Motor.C.forward();
			while (tiltsample[0] < angle) {
				//sampleSize = tilt.sampleSize();
				//tiltsample = new float[sampleSize];
				tilt.getAngleMode().fetchSample(tiltsample, 0);
				System.out.println(tiltsample[0]);
			}
		}
		
		Motor.B.stop(true);
		Motor.C.stop(true);

		tilt.reset();
		wait = 0;
		while(wait<1000)
		{
			wait = wait + 1;
		}
	}
	
	public static void finalAdjust(float angle) throws InterruptedException{
		// set speed
		Motor.B.setSpeed(50); // left
		Motor.C.setSpeed(50); // right
		
		/*
		sampleSize = tilt.sampleSize();
		tiltsample = new float[sampleSize];
		tilt.getAngleMode().fetchSample(tiltsample, 0);
		*/
		
    	//System.out.println(tiltsample[0]);
    	
		
		if (tiltsample[0] > angle ) {
			Motor.B.forward();
			Motor.C.backward();
			while (tiltsample[0] > angle) {
				//sampleSize = tilt.sampleSize();
				//tiltsample = new float[sampleSize];
				tilt.getAngleMode().fetchSample(tiltsample, 0);
				//System.out.println(tiltsample[0]);
			}
		}
		else {
			Motor.B.backward();
			Motor.C.forward();
			while (tiltsample[0] < angle) {
				//sampleSize = tilt.sampleSize();
				//tiltsample = new float[sampleSize];
				tilt.getAngleMode().fetchSample(tiltsample, 0);
				//System.out.println(tiltsample[0]);
			}
		}
		
		Motor.B.stop(true);
		Motor.C.stop(true);

		
	}
	
	public static void pose2pose(node current, node next) throws InterruptedException 
	{
		/*
		sampleSize = tilt.sampleSize();
		tiltsample = new float[sampleSize];
		tilt.getAngleMode().fetchSample(tiltsample, 0);
		*/
		
		float relative_angle = tiltsample[0];
		float goal_angle = 0;
		int dx = next.x - current.x;
		int dy = next.y - current.y;
		
		if(dx == 1){
			goal_angle = 0;	
		}
		else if(dx == -1){
			goal_angle = 180;
		}
		else if(dy == 1){
			goal_angle = 90;
		}
		else if(dy == -1){
			goal_angle = 270; 
		}
		//System.out.println(goal_angle);
		//goal_angle  = goal_angle - true_angle;
		gotoAngle(-1*goal_angle);
		
		
		
		Motor.B.setSpeed(100); // left
		Motor.C.setSpeed(100); // right
		
		Motor.C.rotate(316, true); //318
		Motor.B.rotate(316, true); //318
		Motor.B.waitComplete();
		Motor.C.waitComplete();
	}
	
	public static node[][] graph1()
    {
        int x = 0;
        int y = 0;
        int val = 0;
        node [] [] map = new node [9] [9];
        int row = 0;
        int col = 0;
        for ( row = 0; row < 9; row ++ )
        {
            for ( col = 0; col < 9; col ++ )
            {
                node temp_node = new node(col,row,val);
                map [row] [col] = temp_node;
            }
        }
    
        map[5][0].value = 1;
        for (row = 0; row<4;row++)
        {
            col = 1;
            map[row][col].value = 1;
        }
        map[5][1].value = 1;
        map[7][1].value = 1;
        
        map[7][2].value = 1;
        
        for (row = 1; row<9;row++)
        {
            col = 3;
            map[row][col].value = 1;
        }
        
        map[1][4].value = 1;
        
        map[1][5].value = 1;
        for (row = 3; row<9;row++)
        {
            col = 5;
            map[row][col].value = 1;
        }
        
        map[1][6].value = 1;
        map[3][6].value = 1;
        
        map[1][7].value = 1;
        map[3][7].value = 1;
        for (row = 5; row<8;row++)
        {
            col = 7;
            map[row][col].value = 1;
        }
        
        map[5][8].value = 1;
        
        return map;
    }   
    


	public static void main(String[] args) throws InterruptedException {
		
		/*
		float angle1 = 90;
		float angle2 = 45;
		float angle3 = 90;
		float angle4 = 0;
		gotoAngle(angle1);
		gotoAngle(angle2);
		gotoAngle(angle3);
		gotoAngle(angle4);
		*/
		while (!Button.RIGHT.isDown()) {
			//chill
		}
		
		
		/*
		node start = new node(1,4,0);
        node end = new node(0,4,0);
        
		pose2pose(start,end,270);
		pose2pose(end,start,270);
		*/
		
		//path finding main
		
		//node(x,y,value = 0)
        node start = new node(4,8,0);
        node end = new node(2,8,0);
        int init_angle = 180;
        int final_angle = 90;
        
        node[][] map1 = graph1();
        
        
        
        ArrayList<node> path_found = new ArrayList<node>();
        path_found = path(map1,start,end);
        System.out.println("done planning");
        for (int i = 0; i < path_found.size()-1; i++) {
        	
        	System.out.print("go from: ");
        	System.out.print(path_found.get(i).x);
        	System.out.print(",");
        	System.out.print(path_found.get(i).y);
        	
        	System.out.print("  to: ");
        	System.out.print(path_found.get(i+1).x);
        	System.out.print(",");
        	System.out.print(path_found.get(i+1).y);
        	
        	if(i == 0)
        	{
        		firstAngle(init_angle);
        		pose2pose(path_found.get(i),path_found.get(i+1));
        	}
        	else
        		pose2pose(path_found.get(i),path_found.get(i+1));
        	System.out.print("    Done movement!");
        }
        
        //final adjustment
		finalAdjust(final_angle);
		
		
		
		
	}
}









