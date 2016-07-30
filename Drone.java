package DroneSim;

// Resolving imports

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;

import javax.bluetooth.BluetoothStateException;
import javax.bluetooth.LocalDevice;
import javax.bluetooth.RemoteDevice;
import javax.bluetooth.UUID;
import javax.microedition.io.Connector;
import javax.microedition.io.StreamConnection;
import javax.microedition.io.StreamConnectionNotifier;

public class Drone
{
	private double velocity;
	private Random generator;
	private String chipSet;
	private double latitude;
	private double longitude;
	private int R = 6371000; // The Earth's mean radius in metres
	private double motorWeight = 57; // KDA 20-22L 925KV, there will be 4 of these
	private double propellorWeight = 12; // APC 10x4.7. there will be 4 of these
	private double escWeight = 15; // SuperSimple 18amps Electronic Speed Control, there will be 4 of these
	private double batteryWeight = 188; // 25C 2200mAh
	private double landingSkidGearWeight = 61; // Consists of Landing Skid arms, pipe and nuts
	private double totalWeight;
	// <<< Used to hold data that is calculated through using calculateDistance() method, for single and multiple waypoints >>>
	private static double time = 0;
	private static double timeContinued = 0;
	private static double timeToDecelerate = 0;
	private static double initialVelocity = 0;
	private static double finalVelocity = 0;
	private static double totalTime = 0;
	// <<< Used for Bluetooth connectivity >>>	
	public static ArrayList<String> oldXValues = new ArrayList<String>(); //Stores last values in arrayList
	public static ArrayList<String> oldYValues = new ArrayList<String>();
	
	public static ArrayList<Double> totalX = new ArrayList<Double>();
	public static ArrayList<Double> totalY = new ArrayList<Double>();
	
	
	public static BufferedReader bReader;
	
	public Drone(String chipSet)
	{
		generator = new Random();
		
		// <<< Generate a random point of deployment somewhere within Cornwall Park >>>
		double randDLat = generator.nextDouble();
		double cornwallParkLat = -36.893082;
		double cornwallParkLat2 = -36.898814;
		double range = cornwallParkLat2 - cornwallParkLat;
		double scaled = range * randDLat;
		double randomLatitude = cornwallParkLat + scaled;
		
		double randDLong = generator.nextDouble();
		double cornwallParkLong = 174.787407;
		double cornwallParkLong2 = 174.789767;
		double rangeTwo = cornwallParkLong2 - cornwallParkLong;
		double scaledTwo = rangeTwo * randDLong;
		double randomLongitude = cornwallParkLong + scaledTwo;
		
		latitude = randomLatitude;
		longitude = randomLongitude;
		// <<< >>>
		velocity = 0; // UAV should be at rest when first deployed
		this.chipSet = chipSet;
		
		totalWeight = ((motorWeight * 4) + (propellorWeight * 4) + (escWeight * 4) + batteryWeight + landingSkidGearWeight)/1000;
	}
	
	// Simple accessor and mutator methods used to modify characteristics of the UAV
	
	public double getTotalWeight()
	{
		return totalWeight;
	}
	
	public double getLatitude()
	{
		return latitude;
	}

	public void setLatitude(double latitude)
	{
		this.latitude = latitude;
	}

	public double getLongitude()
	{
		return longitude;
	}

	public void setLongitude(double longitude) 
	{
		this.longitude = longitude;
	}

	public double getVelocity()
	{
		return velocity;
	}
	
	public void setVelocity(double finalVelocity2)
	{
		this.velocity = finalVelocity2;
	}
	
	public String getChipSet()
	{
		return chipSet;
	}
	
	// This uses the "haversine" distance formula to calculate the great-circle distance between
			// two points (shortest distance over the earth's surface) (ignores hills/mountains)
			
	// R is the earth's mean radius
	// Latitude and Longitude should be converted to radians first
	// Formula for haversine, a = sin^2(delta latitude/2) + cos latitude 1 * cos latitude 2 * sin^2(delta longitude/2)
	//						  c = 2 * atan2(square root a, square root 1-a)
	//						  d = R * c	
	
	public double calculateGPSDistance(double latitude1, double longitude1, double latitude2, double longitude2)
	{
		double latitude1AsRadians = Math.toRadians(latitude1);
		double latitude2AsRadians = Math.toRadians(latitude2);
		double deltaLatitude = Math.toRadians(latitude2 - latitude1);
		double deltaLongitude = Math.toRadians(longitude2 - longitude1);
		
		double a = Math.pow(Math.sin(deltaLatitude/2), 2) + Math.cos(latitude1AsRadians) * 
				   Math.cos(latitude2AsRadians) * Math.pow(Math.sin(deltaLongitude/2), 2);
		double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
		double distance = R * c;
		
		return distance; 
	}
	
	// The method for calculating the bearing, the following two methods will be used in conjunction with
	// this method to return an initial bearing or final bearing, the UAV will only use initial bearing for
	// its calculations and displaying of flight parameters, however the final bearing method is available
	// should it ever need to be used in the future
	
	public double calculateBearing(double latitude1, double longitude1, double latitude2, double longitude2)
	{
		double latitude1AsRadians = Math.toRadians(latitude1); // phi 1
		double latitude2AsRadians = Math.toRadians(latitude2); // phi 2
		double longitude1AsRadians = Math.toRadians(longitude1); // lambda 1
		double longitude2AsRadians = Math.toRadians(longitude2); // lambda 2
		double deltaLongitude = longitude2AsRadians - longitude1AsRadians;
		
		double y = Math.sin(deltaLongitude) * Math.cos(latitude2AsRadians);
		double x = Math.cos(latitude1AsRadians) * Math.sin(latitude2AsRadians) - Math.sin(latitude1AsRadians) * Math.cos(latitude2AsRadians)
					* Math.cos(deltaLongitude);
		double theta = Math.atan2(y, x);
		double thetaAsDegrees = Math.toDegrees(theta);
				
		return thetaAsDegrees;
	}
	
	public double getInitialBearing(double latitude1, double longitude1, double latitude2, double longitude2)
	{
		return (calculateBearing(latitude1, longitude1, latitude2, longitude2) + 360) % 360;
	}
	
	// For final bearing, swap the latitude and longitude and then take the angle in the opposite direction
	// (180 degrees around)
	
	public double getFinalBearing(double latitude1, double longitude1, double latitude2, double longitude2)
	{
		return (calculateBearing(latitude2, longitude2, latitude1, longitude1) + 180) % 360;
	}
	
	// Returns a string representation of the cardinal direction based on what the UAV's bearing is,
	// the values used for the if statements were gathered from an online page
	
	public String getNSEW(double theta)
	{
		// The angle increases clockwise 
		String direction = null;
		
		if(theta >= 0 && theta <= 360)
		{
			if(theta < 22.50)
			{
				direction = "North";
			}
			else if(theta < 45)
			{
				direction = "North North East";
			}
			else if(theta < 67.50)
			{
				direction = "North-East";
			}
			else if(theta < 90)
			{
				direction = "East North East";
			}
			else if(theta < 112.50)
			{
				direction = "East";
			}
			else if(theta < 135)
			{
				direction = "East South East";
			}
			else if(theta < 157.50)
			{
				direction = "South-East";
			}
			else if(theta < 180)
			{
				direction = "South South East";
			}
			else if(theta < 202.50)
			{
				direction = "South";
			}
			else if(theta < 225)
			{
				direction = "South South West";
			}
			else if(theta < 247.50)
			{
				direction = "South-West";
			}
			else if(theta < 270)
			{
				direction = "West South West";
			}
			else if(theta < 292.50)
			{
				direction = "West";
			}
			else if(theta < 315)
			{
				direction = "West North West";
			}
			else if(theta < 337.50)
			{
				direction = "North-West";
			}
			else if(theta < 360)
			{
				direction = "North North West";
			}
			else
			{
				direction = "North";
			}
		}
		else
		{
			direction = "Sorry, bearing angle is not valid";
		}
		
		return direction;
	}
	
	// This method that has been commented out was the old implementation of calculateTime, calculateVelocity and calculateDistance, I did not delete it, 
	// just so that it can be used as a reference within the project report
	
	/*public List<Double> calculateTime(double distanceToTravel)
	{
		List<Double> timeValues = new ArrayList<Double>();
		double distanceCalculated = 0;
		double distanceTravelled = 0;
		double distanceLeft = distanceToTravel;
		double time = 0;
		final double timeIntervals = 1;
		double timeToDecelerate = 0;
		double timeContinued = 0;
		//double timeDecelerate = 0; // Used to begin significant deceleration according to 6.5atan(2x) function
		double initialVelocity = 0;
		double finalVelocity = 0;
		//double force = totalWeight * acceleration;
		//double finalVelocitySquared = 0;
		//double initialVelocitySquared = 0;
		//double doubleAcceleration = 2 * acceleration;
		double distanceForAcceleration;
		double distanceForDeceleration;
		double distanceForAccDec;
						
		while(distanceTravelled < distanceToTravel) // Keep running while the UAV has not covered enough distance to reach its destination
		{
			if(distanceToTravel >= 44) // First case, then follows the 10/1+e^-2x+6 sigmoid function, while accelerating it can get upto 5 m/s before decelerating in some cases
			{
				distanceForAcceleration = 30.0;
				distanceForDeceleration = 39.97527376843365;
				distanceForAccDec = 69.97527376843365;
				
				while(distanceTravelled < (distanceToTravel - distanceForDeceleration)) // Perform acceleration and constant speed calculations
				{
					finalVelocity = 10/(1+Math.exp((-2*time) + 6));
					if(time == 0) //Effectively, rounding down to 0 m/s
					{
						finalVelocity = 0;
					}
					if(time >= 6) //Effectively, rounding upto 10 m/s
					{
						finalVelocity = 10;
					}
					
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceTravelled += distanceCalculated;
					initialVelocity = finalVelocity;
				
					if(distanceTravelled >= distanceToTravel -  distanceForDeceleration)
					{
						distanceTravelled = distanceToTravel -  distanceForDeceleration;
						//timeValues.add(time); 
						//time++;
						//timeContinued = time;
						break;
					}
					
					timeValues.add(time);
					time++;
					timeContinued = time;
				}
				
				timeToDecelerate = time; // Taking note of when to start decelerating
				
				finalVelocity = -10/(1+(Math.exp(-2*timeContinued + ((timeToDecelerate*2)+6)))) + 10;
				if(timeContinued >= timeToDecelerate + 6) //Effectively, rounding down to 0 m/s after 6 seconds of deceleration
				{
					finalVelocity = 0;
					distanceTravelled = distanceToTravel; //Failsafe incase distanceTravelled != distanceToTravel by the time vf = 0
				}
				distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
				distanceLeft = distanceToTravel - distanceTravelled;
				
				if(distanceLeft > distanceCalculated)
					distanceTravelled += distanceCalculated;
				else
					distanceTravelled = distanceToTravel;
				
				initialVelocity = finalVelocity;
				timeValues.add(timeContinued);
				timeContinued++;
			}
			else if(distanceToTravel >= 17) // 2nd case, follows 5/1+e^-2x+4 sigmoid function, while accelerating it can get upto 2.5 m/s before decelerating in some cases
			{
				distanceForAcceleration = 10.0;
				distanceForDeceleration = 14.91006895018954;
				distanceForAccDec = 24.91006895018954;
				
				while(distanceTravelled < (distanceToTravel - distanceForDeceleration))
				{
					finalVelocity = 5/(1+Math.exp((-2*time) + 4));
					if(time == 0) //Effectively, rounding down to 0 m/s
					{
						finalVelocity = 0;
					}
					if(time >= 4) //Effectively, rounding upto 5 m/s
					{
						finalVelocity = 5;
					}
					
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceTravelled += distanceCalculated;
					initialVelocity = finalVelocity;
				
					if(distanceTravelled >= distanceToTravel -  distanceForDeceleration)
					{
						distanceTravelled = distanceToTravel -  distanceForDeceleration;
						//timeValues.add(time); 
						//time++;
						//timeContinued = time;
						break;
					}
					
					timeValues.add(time);
					time++;
					timeContinued = time;
				}
				
				timeToDecelerate = time;
				
				finalVelocity = -5/(1+(Math.exp(-2*timeContinued + ((timeToDecelerate*2)+4)))) + 5;
				if(timeContinued >= timeToDecelerate + 4) //Effectively, rounding down to 0 m/s after 4 seconds of deceleration
				{
					finalVelocity = 0;
					distanceTravelled = distanceToTravel; //Failsafe incase distanceTravelled != distanceToTravel by the time vf = 0
				}
				distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
				distanceLeft = distanceToTravel - distanceTravelled;
				
				if(distanceLeft > distanceCalculated)
					distanceTravelled += distanceCalculated;
				else
					distanceTravelled = distanceToTravel;
				
				initialVelocity = finalVelocity;
				timeValues.add(timeContinued);
				timeContinued++;
				//timeDecelerate++;
			}
			else // 3rd case, follows 3/1+e^-2x + 3 sigmoid function, while accelerating it can get upto 0.806 m/s before decelerating in some cases
			{
				distanceForAcceleration = 4.5;
				distanceForDeceleration = 7.357722380467299;
				distanceForAccDec = 11.8577223804673;
				
				while(distanceTravelled < (distanceToTravel - distanceForDeceleration))
				{
					finalVelocity = 3/(1+Math.exp((-2*time) + 3));
					if(time == 0) //Effectively, rounding down to 0 m/s
					{
						finalVelocity = 0;
					}
					if(time >= 3) //Effectively, rounding upto 3 m/s
					{
						finalVelocity = 3;
					}
					
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceTravelled += distanceCalculated;
					initialVelocity = finalVelocity;
				
					if(distanceTravelled >= distanceToTravel -  distanceForDeceleration)
					{
						distanceTravelled = distanceToTravel -  distanceForDeceleration;
						//timeValues.add(time); 
						//time++;
						//timeContinued = time;
						break;
					}
					
					timeValues.add(time);
					time++;
					timeContinued = time;
				}
				
				timeToDecelerate = time;
				
				finalVelocity = -3/(1+(Math.exp(-2*timeContinued + ((timeToDecelerate*2)+3)))) + 3;
				
				if(timeContinued == 0)
				{
					finalVelocity = 0;
				}
				
				if(timeContinued >= timeToDecelerate + 3) //Effectively, rounding down to 0 m/s after 3 seconds of deceleration
				{
					finalVelocity = 0;
					distanceTravelled = distanceToTravel; //Failsafe incase distanceTravelled != distanceToTravel by the time vf = 0
				}
				distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
				distanceLeft = distanceToTravel - distanceTravelled;
				
				if(distanceLeft > distanceCalculated)
					distanceTravelled += distanceCalculated;
				else
					distanceTravelled = distanceToTravel;
				
				initialVelocity = finalVelocity;
				timeValues.add(timeContinued);
				timeContinued++;
			}
		}
		
		return timeValues;
	} */
	
	// The modified version of calculateDistance, this method will perform the appropriate calculations that will be used
	// to return different flight parameters and used with other calculation methods
	
	public double calculateDistance(double distanceToTravel, double distanceTravelled)
	{
		double distanceCalculated = 0;
		double distanceLeft = distanceToTravel;
		final double timeIntervals = 1; // This is used when calculating the distance travelled by the UAV during 1 second time intervals
		double distanceForAcceleration; // Not needed by this method, just given the calculated value according to test programs
		double distanceForDeceleration;
		double distanceForAccDec; // Not needed by this method, just given the calculated value according to test programs
		double distanceToReturn;
			
		// The while loop has been replaced since that will now be used in the main method
		
			if(distanceToTravel >= 44) // First case, then follows the 10/1+e^-2x+6 sigmoid function, while accelerating it can get upto 5 m/s before decelerating in some cases
			{
				distanceForAcceleration = 30.0;
				distanceForDeceleration = 39.97527376843365;
				distanceForAccDec = 69.97527376843365;
				
				if(distanceTravelled < (distanceToTravel - distanceForDeceleration)) 
				{
					finalVelocity = 10/(1+Math.exp((-2*time) + 6));
					if(time == 0) //Effectively, rounding down to 0 m/s
					{
						finalVelocity = 0;
					}
					if(time >= 6) //Effectively, rounding upto 10 m/s
					{
						finalVelocity = 10;
					}
					
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceTravelled += distanceCalculated;
					initialVelocity = finalVelocity;
				
					if(distanceTravelled >= distanceToTravel -  distanceForDeceleration)
					{
						distanceTravelled = distanceToTravel -  distanceForDeceleration;
					}
					
					distanceToReturn = distanceTravelled;
					time++; // Time instance variable modified so that during the next iteration of the method within the while loop which is placed in the main method, the correct distance value
							// is calculated
					timeContinued = time;
				}
				else
				{
					timeToDecelerate = time;
				
					finalVelocity = -10/(1+(Math.exp(-2*timeContinued + ((timeToDecelerate*2)+6)))) + 10;
					if(timeContinued >= timeToDecelerate + 6) // Effectively, rounding down to 0 m/s after 6 seconds of deceleration
					{
						finalVelocity = 0;
						distanceTravelled = distanceToTravel; // Failsafe incase distanceTravelled != distanceToTravel by the time vf = 0
					}
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceLeft = distanceToTravel - distanceTravelled;
					
					if(distanceLeft > distanceCalculated) // Keep adding onto distanceTravelled if the UAV has not arrived at its destination
						distanceTravelled += distanceCalculated;
					else
						distanceTravelled = distanceToTravel;
					
					initialVelocity = finalVelocity;
					distanceToReturn = distanceTravelled;
					timeContinued++; // timeContinued will replace the use of the time static variable for the deceleration phase of the calculations
				}	
			}
			else if(distanceToTravel >= 17) // 2nd case, follows 5/1+e^-2x+4 sigmoid function, while accelerating it can get upto 2.5 m/s before decelerating in some cases
			{
				distanceForAcceleration = 10.0;
				distanceForDeceleration = 14.91006895018954;
				distanceForAccDec = 24.91006895018954;
				
				if(distanceTravelled < (distanceToTravel - distanceForDeceleration))
				{
					finalVelocity = 5/(1+Math.exp((-2*time) + 4));
					if(time == 0) //Effectively, rounding down to 0 m/s
					{
						finalVelocity = 0;
					}
					if(time >= 4) //Effectively, rounding upto 5 m/s
					{
						finalVelocity = 5;
					}
					
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceTravelled += distanceCalculated;
					initialVelocity = finalVelocity;
				
					if(distanceTravelled >= distanceToTravel -  distanceForDeceleration)
					{
						distanceTravelled = distanceToTravel -  distanceForDeceleration;
					}
					
					distanceToReturn = distanceTravelled;
					time++;
					timeContinued = time;
				}
				else
				{	
					timeToDecelerate = time;
					
					finalVelocity = -5/(1+(Math.exp(-2*timeContinued + ((timeToDecelerate*2)+4)))) + 5;
					if(timeContinued >= timeToDecelerate + 4) // Effectively, rounding down to 0 m/s after 4 seconds of deceleration
					{
						finalVelocity = 0;
						distanceTravelled = distanceToTravel; // Failsafe incase distanceTravelled != distanceToTravel by the time vf = 0
					}
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceLeft = distanceToTravel - distanceTravelled;
					
					if(distanceLeft > distanceCalculated) // Keep adding onto distanceTravelled if the UAV has not arrived at its destination
						distanceTravelled += distanceCalculated;
					else
						distanceTravelled = distanceToTravel;
					
					initialVelocity = finalVelocity;
					distanceToReturn = distanceTravelled;
					timeContinued++;
				}	
			}
			else // 3rd case, follows 3/1+e^-2x + 3 sigmoid function, while accelerating it can get upto 0.806 m/s before decelerating in some cases
			{
				distanceForAcceleration = 4.5;
				distanceForDeceleration = 7.357722380467299;
				distanceForAccDec = 11.8577223804673;
				
				if(distanceTravelled < (distanceToTravel - distanceForDeceleration))
				{
					finalVelocity = 3/(1+Math.exp((-2*time) + 3));
					if(time == 0) //Effectively, rounding down to 0 m/s
					{
						finalVelocity = 0;
					}
					if(time >= 3) //Effectively, rounding upto 3 m/s
					{
						finalVelocity = 3;
					}
					
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceTravelled += distanceCalculated;
					initialVelocity = finalVelocity;
				
					if(distanceTravelled >= distanceToTravel -  distanceForDeceleration)
					{
						distanceTravelled = distanceToTravel -  distanceForDeceleration;
					}
					
					distanceToReturn = distanceTravelled;
					time++;
					timeContinued = time;
				}
				else
				{	
					timeToDecelerate = time;
					
					finalVelocity = -3/(1+(Math.exp(-2*timeContinued + ((timeToDecelerate*2)+3)))) + 3;
					
					if(timeContinued == 0) // If distanceToTravel is lesser than distanceForDeceleration, then set velocity to 0 m/s when the time equals zero  
					{
						finalVelocity = 0;
					}
					
					if(timeContinued >= timeToDecelerate + 3) // Effectively, rounding down to 0 m/s after 3 seconds of deceleration
					{
						finalVelocity = 0;
						distanceTravelled = distanceToTravel; // Failsafe incase distanceTravelled != distanceToTravel by the time vf = 0
					}
					distanceCalculated = (initialVelocity + finalVelocity)*timeIntervals/2;
					distanceLeft = distanceToTravel - distanceTravelled;
					
					if(distanceLeft > distanceCalculated) // Keep adding onto distanceTravelled if the UAV has not arrived at its destination
						distanceTravelled += distanceCalculated;
					else
						distanceTravelled = distanceToTravel;
					
					initialVelocity = finalVelocity;
					distanceToReturn = distanceTravelled;
					timeContinued++;
				}	
			}
			this.setVelocity(finalVelocity); // Set the UAV's local velocity variable as what was calculated according to the method 
			return distanceToReturn;
		}
	
	// Given the starting latitude, the initial bearing and the distance travelled, a destination point can be calculated. As distanceTravelled shall keep increasing
	// while the UAV is travelling from one point to another, during each second the next latitude along the UAV's flight path can be calculated using this method.
	
	public double latitudeTravelled(double startingLatitude, double initialBearing, double distanceTravelled)
	{
		double angularDistance;
		double startingLatAsRadians = Math.toRadians(startingLatitude);
		double initialBearingAsRadians = Math.toRadians(initialBearing);
		double updatedLatitude;
				
			angularDistance = distanceTravelled/R;
			updatedLatitude = Math.asin(Math.sin(startingLatAsRadians) * Math.cos(angularDistance) + Math.cos(startingLatAsRadians) * Math.sin(angularDistance) 
										* Math.cos(initialBearingAsRadians));
			
		this.setLatitude(Math.toDegrees(updatedLatitude));	
		return Math.toDegrees(updatedLatitude);
	}
	
	// Given the starting latitude, starting longitude, the initial bearing, the distance travelled and the latitude along the UAV's flight path, a destination point can be calculated. 
	// As distanceTravelled shall keep increasing while the UAV is travelling from one point to another, during each second the next latitude along the UAV's flight path can be calculated 
	// using this method.
	
	public double longitudeTravelled(double startingLatitude, double startingLongitude, double initialBearing, double distanceTravelled, double intermediateLatitude)
	{
		double angularDistance;
		double startingLatAsRadians = Math.toRadians(startingLatitude);
		double startingLongAsRadians = Math.toRadians(startingLongitude);
		double initialBearingAsRadians = Math.toRadians(initialBearing);
		double intermediateLatAsRadians = Math.toRadians(intermediateLatitude);
		double updatedLongitude;
		
			angularDistance = distanceTravelled/R;
			updatedLongitude = startingLongAsRadians + Math.atan2(Math.sin(initialBearingAsRadians) * Math.sin(angularDistance) * Math.cos(startingLatAsRadians), 
											Math.cos(angularDistance) - Math.sin(startingLatAsRadians) * Math.sin(intermediateLatAsRadians));
				
		this.setLongitude(Math.toDegrees(updatedLongitude));	
	    return Math.toDegrees(updatedLongitude);  
	}
	
	// MULTIPLE WAYPOINTS methods follows...
	
	// This method can be used when dealing with waypoints
	// Used to calculate distance between waypoints, the list parameters should contain values for the lat and long
	// for each waypoint set by the user on the android app (NOT each lat and long value that the UAV has travelled)
		
	public List<Double> calculateDistanceBetweenWaypoints(List<Double> latitudes, List<Double> longitudes)
	{
		double currentLat = this.getLatitude();
		double currentLong = this.getLongitude();
		double intermediateLat;
		double intermediateLong;
		double distance;
		List<Double> distancesLeft = new ArrayList<Double>();
		
		for(int i = 0; i < latitudes.size(); i++)
		{
			intermediateLat = latitudes.get(i);
			intermediateLong = longitudes.get(i);
			
			distance = this.calculateGPSDistance(currentLat, currentLong, intermediateLat, intermediateLong);
			
			distancesLeft.add(distance);
			
			currentLat = intermediateLat;
			currentLong = intermediateLong;
		}
		
		return distancesLeft;
	}
	
	// Calculate bearings between waypoints
	
	public List<Double> calculateBearingBetweenWaypoints(List<Double> latitudes, List<Double> longitudes)
	{
		double currentLat = this.getLatitude();
		double currentLong = this.getLongitude();
		double intermediateLat;
		double intermediateLong;
		double bearing;
		List<Double> bearings = new ArrayList<Double>();
		
		for(int i = 0; i < latitudes.size(); i++)
		{
			intermediateLat = latitudes.get(i);
			intermediateLong = longitudes.get(i);
			
			bearing = this.getInitialBearing(currentLat, currentLong, intermediateLat, intermediateLong);
			
			bearings.add(bearing);
			
			currentLat = intermediateLat;
			currentLong = intermediateLong;
		}
		
		return bearings;
	}
	
	// Simply returns a total distance that needs to be travelled by the UAV when travelling through multiple waypoints
	
	public double calculateTotalDistanceWaypoints(List<Double> latitudes, List<Double> longitudes)
	{
		double totalDistance = 0;
		double currentLat = this.getLatitude();
		double currentLong = this.getLongitude();
		double intermediateLat;
		double intermediateLong;
		
		for(int i = 0; i < latitudes.size(); i++)
		{
			intermediateLat = latitudes.get(i);
			intermediateLong = longitudes.get(i);
			
			totalDistance += this.calculateGPSDistance(currentLat, currentLong, intermediateLat, intermediateLong);
									
			currentLat = intermediateLat;
			currentLong = intermediateLong;
		}
		
		return totalDistance;
	}
	
	//*Heemesh*	Server Setup
    
	private static StreamConnection serverSetup() throws IOException
    {
        //Create a UUID for SPP
        UUID uuid = new UUID("1101", true);
        //Create the servicve url
        String connectionString = "btspp://localhost:" + uuid +";name=Sample SPP Server";

        //Open server url
        StreamConnectionNotifier streamConnNotifier = (StreamConnectionNotifier)Connector.open(connectionString);

        //Wait for client connection
        System.out.println("\nServer Started. Waiting for clients to connect...");
        StreamConnection connection=streamConnNotifier.acceptAndOpen();

        RemoteDevice dev = RemoteDevice.getRemoteDevice(connection);
        System.out.println("Remote device address: "+dev.getBluetoothAddress());
        
        return connection;
    }
    
    //*Heemesh*
    
	private static List[] receive() throws IOException
    {	        
        String lineRead = bReader.readLine();	//String received
        
        System.out.println("Received data: " + lineRead);
        
        String[] parts = lineRead.split(":");	//Splits string by ':'
        String xWayPoints = parts[0];
		String yWayPoints = parts[1];
		List<String> xValuesL = Arrays.asList(xWayPoints.split(","));	//Splits both strings
		List<String> yValuesL = Arrays.asList(yWayPoints.split(","));	//by ','
		
		ArrayList<String> xValuesAL = new ArrayList<String>(xValuesL);
		ArrayList<String> yValuesAL = new ArrayList<String>(yValuesL);
		
		for (int i = 0; i < xValuesAL.size(); i++) {
			double con = Double.parseDouble(xValuesAL.get(i));
			totalX.add(con);
			
			double con2 = Double.parseDouble(yValuesAL.get(i));
			totalY.add(con2);
		}
		
		//Checks old points
		List<String> xValList = new ArrayList<String>(xValuesAL);
		xValList.removeAll(oldXValues);
		List<String> yValList = new ArrayList<String>(yValuesAL);
		yValList.removeAll(oldYValues);
		
		//Adds new points to the 'old' ArrayLists
		for(int i=0; i<xValList.size(); i++)
	    {
			oldXValues.add(xValList.get(i));
	    }
		for(int i=0;i<yValList.size();i++)
	    {
			oldYValues.add(yValList.get(i));
	    }
		
		ArrayList<Double> finalXValues = new ArrayList<Double>();
		ArrayList<Double> finalYValues = new ArrayList<Double>();
		ArrayList<Boolean> trueflase = new ArrayList<Boolean>();
		
		//Converts strings to double
		for(int i=0; i<xValList.size(); i++)
		{
			String strConvert1 = xValList.get(i);
			double dValue1 = Double.parseDouble(strConvert1);
			String strConvert2 = yValList.get(i);
			double dValue2 = Double.parseDouble(strConvert2);
			
			finalXValues.add(dValue1);
			finalYValues.add(dValue2);
		}
		
		//Checks if single or multiple waypoint
		boolean singleMulti;
        if(finalXValues.size() == 1){
        	singleMulti = true;
        }
        else{
        	singleMulti = false;
        }
        trueflase.add(singleMulti);
		
        //Returns x and y values as well as the single or multi point 
        return new List[] {finalXValues, finalYValues, trueflase};
    }
	
	//*Heemesh*
    
	private static void sendSingleP(PrintWriter pWriter, 							
    		double n1, double n2, double n3, double n4, double n5, double n6) 							
    {
        String no1 = String.valueOf(n1);	//Converts all double values to string
        String no2 = String.valueOf(n2);
        String no3 = String.valueOf(n3);
        String no4 = String.valueOf(n4);
        String no5 = String.valueOf(n5);
        String no6 = String.valueOf(n6);
        
        //Connect up both strings to send
        //Adds 'S' to stand for single point
        String finalSendOut = "S" + no1 + "," + no2 + "," + no3 + "," + no4 + "," + no5 + "," + no6 + "\n";
        
        pWriter.write(finalSendOut);	//Writes string to Print writer to send
        System.out.println(finalSendOut);
        pWriter.flush();
    }
    
    //*Heemesh*
    private static void sendMultipleP(PrintWriter pWriter, 
    		double n1, double n2, double n3, double n4, double n5, double n6, double n7)	
    {
        String no1 = String.valueOf(n1);	//Converts all double values to string
        String no2 = String.valueOf(n2);
        String no3 = String.valueOf(n3);
        String no4 = String.valueOf(n4);
        String no5 = String.valueOf(n5);
        String no6 = String.valueOf(n6);
        String no7 = String.valueOf(n7);
        
        //Connect up both strings to send
        //Adds 'M' to stand for multiple point
        String finalSendOut = "M" + no1 + "," + no2 + "," + no3 + "," + no4 + "," + no5 + "," + no6 + "," + no7 + "\n";
        
        pWriter.write(finalSendOut);	//Writes string to Print writer to send
        System.out.println(finalSendOut);
        pWriter.flush();
    }
    
    //*Heemesh*
    private static void sendBearing(PrintWriter pWriter, 
    		double n1, double n2) 															
    {
        String no1 = String.valueOf(n1);//Converts both double values to string
        String no2 = String.valueOf(n2);

        String finalSendOut = "B" + no1 + "," + no2 + "\n";	//Connect up both strings to send
															//Adds 'B' to stand for bearing point
        pWriter.write(finalSendOut);	//Writes string to Print writer to send
        System.out.println(finalSendOut);
        pWriter.flush();
    }
	
    //*Heemesh*
    private static void sendStartPosition(PrintWriter pWriter, 
    		double n1, double n2) 															
    {
        String no1 = String.valueOf(n1);	//Converts both double values to string
        String no2 = String.valueOf(n2);

        String finalSendOut = "I" + no1 + "," + no2 + "\n";	//Connect up both strings to send.
        													//Adds 'I' to stand for initial points
        pWriter.write(finalSendOut);	//Writes string to Print writer to send
        System.out.println(finalSendOut);
        pWriter.flush();			
    }
    
	public static void main(String[] args)
	{
		Drone drone = new Drone("Atmel"); // Instantiating Drone object
		//*Heemesh* Initialize variables
		LocalDevice localDevice;
		StreamConnection connection = null;	
		OutputStream outstream;
		PrintWriter pWriter = null;
		
		try // Try block used to execute code relating to the Bluetooth connectivity and the Thread.sleep() timer
		{
			// *Heemesh* Prints Address and Name of current devices Bluetooth
			localDevice = LocalDevice.getLocalDevice();
			System.out.println("Address: " + localDevice.getBluetoothAddress());
			System.out.println("Name: " + localDevice.getFriendlyName());

			//*Heemesh* Bluetooth Setup
			connection = serverSetup();
			// Send response to client
			outstream = connection.openOutputStream();
			pWriter = new PrintWriter(new OutputStreamWriter(outstream));
			//Input stream setup for receiving data
			InputStream inStream=connection.openInputStream();
	        bReader=new BufferedReader(new InputStreamReader(inStream));

	        // System.out.println() statements will be used if different areas of the main method so that data can also be displayed
	        // on the Eclipse console for someone viewing the Java program while it is being run
	        
			System.out.println("The drone has been deployed from these coordinates");
			System.out.println("The drone's latitude is: " + drone.getLatitude());
			System.out.println("The drone's longitude is: " + drone.getLongitude());
			System.out.println("<<<<<-------------------------------------------------->>>>>");
			sendStartPosition(pWriter, drone.getLatitude(), drone.getLongitude());	//*Heemesh* Passes the longitude and
																				// latitude of initial drone postion
			
			while(true) // This while loop will be used to allow the program to keep running, even after performing a first set of computations
			{
				// Declaring and Initializing local variables that will be used to store the results of the calculations pertaining to the UAV's flight parameters
				double distanceTravelled = 0;
				double latitudesV2;
				double longitudesV2;
				double distanceT;
				double timeToSend;
				double currentStartingLatitude;
				double currentStartingLongitude;
				// Declaring variables that will be used to store a rounded result of the calculations, these will be the values sent to the Android app via Bluetooth
				double timeInMain;
				double vel;
				double dist;
				double bear;
				double totalDist;
				double xArrayLSize;
				double yArrayLSize;
				
				//*Heemesh* Receive longitude and latitude values
				List[] ListXY = receive();
				ArrayList<Double> xArrayL = new ArrayList<Double>(ListXY[0]); // Stores the latitudes of the waypoint set by the user
				ArrayList<Double> yArrayL = new ArrayList<Double>(ListXY[1]); // Stores the longitudes of the waypoint set by the user
				ArrayList<Boolean> singleOrMulti = new ArrayList<Boolean>(ListXY[2]); // // Used to determine whether the UAV needs to travel to a single waypoint or multiple waypoints
				System.out.println("True if single, False if Multiple: " + singleOrMulti.get(0));
				
				// Single Point
				if (singleOrMulti.get(0)) 
				{	 
					System.out.println("<<<<<BEFORE SINGLE POINT TRAVEL>>>>>");
					System.out.println("The drone's current position is: ");
					System.out.println("The drone's latitude is: " + drone.getLatitude());
					System.out.println("The drone's longitude is: " + drone.getLongitude());

					currentStartingLatitude = drone.getLatitude();
					currentStartingLongitude= drone.getLongitude();
					double destinationLatitude = xArrayL.get(0); // Retrieve only the first element in this arraylist as this is the only latitude of a single waypoint
					double destinationLongitude = yArrayL.get(0); // Retrieve only the first element in this arraylist as this is the only longitude of a single waypoint
					double distanceToTravel = drone.calculateGPSDistance(currentStartingLatitude, currentStartingLongitude, destinationLatitude, destinationLongitude);	
					double bearing = drone.getInitialBearing(currentStartingLatitude, currentStartingLongitude, destinationLatitude, destinationLongitude);
					System.out.println("<<<<< The distance to travel is: " + distanceToTravel);
					Thread.sleep(5000);
					while(distanceTravelled < distanceToTravel) // The while loop used to continuously calculate and send flight parameter data to the Android app while the UAV hasn't arrived
					{											// at its destination
						distanceT = drone.calculateDistance(distanceToTravel, distanceTravelled);
						timeToSend = timeContinued - 1; // timeContinued, not "time" instance variable is used here since "time" needs to be set as zero after performing first calculation before
														// being iterated because with the sigmoid function that simulates velocity, when t = 0, v != 0
						latitudesV2 = drone.latitudeTravelled(currentStartingLatitude, bearing, distanceT); 
						longitudesV2 = drone.longitudeTravelled(currentStartingLatitude, currentStartingLongitude, bearing, distanceT, latitudesV2);
						System.out.println("The time at this timestep is: " + timeToSend);
						System.out.println("The velocity at this timestep is: " + drone.getVelocity());
						System.out.println("The distance travelled at this timestep is: " + distanceT);
						System.out.println("The latitude at this timestep is: " + drone.getLatitude());
						System.out.println("The longitude at this timestep is: " + drone.getLongitude());
						System.out.println("The bearing towards this destination is: " + bearing +  " " + drone.getNSEW(bearing));
						System.out.println("-----------------------------------------------------------------------");
						distanceTravelled = distanceT;
						
						// These variables will be sent as they are rounded values
						
						timeInMain = (double) Math.round(timeToSend * 100d) / 100d;
						vel = (double) Math.round(drone.getVelocity() * 100d) / 100d;
						dist = (double) Math.round(distanceT * 100d) / 100d;
						bear = (double) Math.round(bearing * 100d) / 100d;
						
						
						sendSingleP(pWriter, timeInMain, vel, dist, latitudesV2, longitudesV2, bear); //*Heemesh* Passes six values
																									  //to sendSingleP method
						Thread.sleep(1000); // Delays the program by 1 second
					}
					
					System.out.println("You have reached your destination.");
					System.out.println(":)");
					// Instance variables are cleared to allow for correct calculations for the next set of input 
					time = 0;
					timeContinued = 0;
					timeToDecelerate = 0;
					initialVelocity = 0;
					finalVelocity = 0;
					distanceTravelled = 0;
					// The arraylists for latitude and longitude are cleared to allow for new values pertaining to the next set of waypoints that are set by the user of the Android app
					singleOrMulti.clear();
					xArrayL.clear();
					yArrayL.clear();
				}
				else // Multiple Points
				{
					currentStartingLatitude = drone.getLatitude(); // Updating to current lat
					currentStartingLongitude= drone.getLongitude(); // Updating to current long
					System.out.println("<<<<<BEFORE MULTIPLE WAYPOINT TRAVEL>>>>>");
					System.out.println("The drone's current position is: ");
					System.out.println("The drone's latitude is: " + drone.getLatitude());
					System.out.println("The drone's longitude is: " + drone.getLongitude());
					Thread.sleep(5000);
			
					double totalDistance = 0;
					double currentDistanceTravelled = 0;
					double intermediateDistanceTravelled;
					double differenceInDistance;
					double currentDistance;
					double currentBearing;
					double intermediateBearing;
					double differenceInBearing;
					
					List<Double> distances = drone.calculateDistanceBetweenWaypoints(xArrayL, yArrayL); 
					List<Double> bearings = drone.calculateBearingBetweenWaypoints(xArrayL, yArrayL); 
						
					for(int i = 0; i < distances.size(); i++) // Used to obtain each distance that needs to be travelled and bearing for each waypoint 
					{
						currentDistance = distances.get(i);
						currentBearing = bearings.get(i);
						while(distanceTravelled < currentDistance) // The while loop used to continuously calculate and send flight parameter data to the Android app while the UAV hasn't arrived
						{										   // at its destination	
							intermediateDistanceTravelled = drone.calculateDistance(currentDistance, distanceTravelled);
							latitudesV2 = drone.latitudeTravelled(currentStartingLatitude, currentBearing, intermediateDistanceTravelled); 
							longitudesV2 = drone.longitudeTravelled(currentStartingLatitude, currentStartingLongitude, currentBearing, intermediateDistanceTravelled, latitudesV2);
							System.out.println("The time at this timestep is: " + totalTime); // totalTime instance variable will be used to monitor time for multiple way points as
																							  // this will also keep track of the time to recalibrate the UAV's bearing towards the next waypoint 	
							System.out.println("The velocity at this timestep is: " + drone.getVelocity());
							System.out.println("The distance travelled at this timestep is: " + intermediateDistanceTravelled);
							System.out.println("The latitude at this timestep is: " + drone.getLatitude());
							System.out.println("The longitude at this timestep is: " + drone.getLongitude());
							System.out.println("The bearing for this flightpath is: " + currentBearing + " " + drone.getNSEW(currentBearing));
							System.out.println("-----------------------------------------------------------------------");
							differenceInDistance = intermediateDistanceTravelled - currentDistanceTravelled;
							totalDistance += differenceInDistance;
							System.out.println("The total distance travelled is: " + totalDistance); // This will continue to keep track of the total distance travelled between waypoints
							System.out.println("<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>");
							distanceTravelled = intermediateDistanceTravelled;
							
							// These variables will be sent as they are rounded values
							
							timeInMain = (double) Math.round(totalTime * 100d) / 100d;
							vel = (double) Math.round(drone.getVelocity() * 100d) / 100d;
							dist = (double) Math.round(intermediateDistanceTravelled * 100d) / 100d;
							bear = (double) Math.round(currentBearing * 100d) / 100d;
							totalDist = (double) Math.round(totalDistance * 100d) / 100d;
							
							sendMultipleP(pWriter, timeInMain, vel, dist, 				//*Heemesh* Passes seven double 
									latitudesV2, longitudesV2, bear, totalDist); 		//values to sendMultipleP method
							Thread.sleep(1000);
							totalTime++;
							currentDistanceTravelled = intermediateDistanceTravelled;
						}	
						
						currentStartingLatitude = drone.getLatitude(); // Updating to current latitude
						currentStartingLongitude = drone.getLongitude(); // Updating to current longitude
								
						if(i < bearings.size() - 1)
						{
							System.out.println("Recalibrating bearing towards next waypoint...");
							System.out.println("-----------------------------------------------------------------------");
							
							intermediateBearing = bearings.get(i+1);
									
							// Assuming the UAV can change its bearing by 40 degrees every second
							// 1/40 = 0.025 s = 25 ms
							// 25 ms to change bearing by 1 degree		
							if(intermediateBearing > currentBearing)
							{	
								differenceInBearing = intermediateBearing - currentBearing;
								while(currentBearing < intermediateBearing)
								{
									currentBearing++;
									Thread.sleep(25);
									totalTime += 0.025;
									System.out.println("The bearing is: " + currentBearing);
									System.out.println("The total time is: " + totalTime);
									System.out.println("-----------------------------------------------------------------------");
									timeInMain = (double) Math.round(totalTime * 100d) / 100d;
									bear = (double) Math.round(currentBearing * 100d) / 100d;
									sendBearing(pWriter, bear, timeInMain); //*Heemesh* Passes two double values to sendBearingP method
								}
							}
							else
							{
								differenceInBearing = currentBearing - intermediateBearing;
								while(currentBearing > intermediateBearing)
								{
									currentBearing--;
									Thread.sleep(25);
									totalTime += 0.025;
									System.out.println("The bearing is: " + currentBearing);
									System.out.println("The total time is: " + totalTime);
									System.out.println("-----------------------------------------------------------------------");
									timeInMain = (double) Math.round(totalTime * 100d) / 100d;
									bear = (double) Math.round(currentBearing * 100d) / 100d;
									sendBearing(pWriter, bear, timeInMain); //*Heemesh* Passes two double values to sendBearingP method
								}
							}	
						}
						// Instance variables are cleared to allow for correct calculations for the next set of input 
						time = 0;
						timeContinued = 0;
						timeToDecelerate = 0;
						initialVelocity = 0;
						finalVelocity = 0;
						distanceTravelled = 0;
						currentDistanceTravelled = 0;
					}	
					
					time = 0;		
					System.out.println("You have reached your destination.");
					System.out.println(":)");
					// The arraylists for latitude and longitude are cleared to allow for new values pertaining to the next set of waypoints that are set by the user of the Android app
					singleOrMulti.clear();
					xArrayL.clear();
					yArrayL.clear();
					totalTime = 0;
				} 	
									
			}
			
		}
		// Catch blocks used to handle potential exception errors that occur with the program, namely for Bluetooth connectivity and Thread.sleep() timer
		catch(InterruptedException e)
		{
			e.printStackTrace();
		}
		catch (IOException e) 
		{
			e.printStackTrace();
		}
	}
}	

