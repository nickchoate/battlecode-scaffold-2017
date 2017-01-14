package nickchoate;

import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import battlecode.common.*;

public strictfp class RobotPlayer {
    static RobotController rc;

    /**
     * run() is the method that is called when a robot is instantiated in the Battlecode world.
     * If this method returns, the robot dies!
    **/
    @SuppressWarnings("unused")
    public static void run(RobotController rc) throws GameActionException {

        // This is the RobotController object. You use it to perform actions from this robot,
        // and to get information on its current status.
        RobotPlayer.rc = rc;

        // Here, we've separated the controls into a different method for each RobotType.
        // You can add the missing ones or rewrite this into your own control structure.
        switch (rc.getType()) {
            case ARCHON:
                runArchon();
                break;
            case GARDENER:
                runGardener();
                break;
            case SOLDIER:
                runSoldier();
                break;
            case LUMBERJACK:
                runLumberjack();
                break;
        }
	}
    
    public final static int ARCHON_CHANNEL_X = 0;
    public final static int ARCHON_CHANNEL_Y = 0;

    static void runArchon() throws GameActionException {
        System.out.println("I'm an archon!");
        int numGardenersCreated = 0;
        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {
            	
                // Generate a random direction
                Direction dir = randomDirection();

                // Randomly attempt to build a gardener in this direction
                boolean makeMore = false;
                if(numGardenersCreated < 6)
                {
                	makeMore = true;
                }
                if(Math.random() < .01)
            	{
            		makeMore = true;
            	}
                if (rc.canHireGardener(dir)  && makeMore) {
                	System.out.println("Build gardener");
                    rc.hireGardener(dir);
                    numGardenersCreated++;
                }
                else
                {
                	float totalBullets = rc.getTeamBullets();
                	if(totalBullets > 1000)
                	{
                		rc.donate(100);
                	}
                }

                // Move randomly
                //tryMove(randomDirection());

                // Broadcast archon's location for other robots on the team to know
                MapLocation myLocation = rc.getLocation();
                rc.broadcast(ARCHON_CHANNEL_X,(int)myLocation.x);
                rc.broadcast(ARCHON_CHANNEL_Y,(int)myLocation.y);

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Archon Exception");
                e.printStackTrace();
            }
        }
    }

	static void runGardener() throws GameActionException {
        System.out.println("I'm a gardener!");
        RobotType lastRobotCreated = null; //0 = solider 1 = lumberjack
        Team friendly = rc.getTeam();
        int randomUpperLimit = 7;
        Random rand = new Random(1321354986413516454L);
        int robotsCreated =0;
        int lumberjacksCreated = 0;
        int solidersCreated = 0;
        int numRobotsToMake = 2;
        int numTreesPlanted = 0;
        
        boolean johnnyAppleseed = true;
        if(rand.nextInt()%2 == 0)
        {
        	johnnyAppleseed = false;
        }
        
        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {

                // Listen for home archon's location
                int xPos = rc.readBroadcast(0);
                int yPos = rc.readBroadcast(1);
                MapLocation archonLoc = new MapLocation(xPos,yPos);

                // Generate a random direction
                Direction dir = randomDirection();
       
                // Randomly attempt to build a soldier or lumberjack in this direction
                boolean builtSomething = false;
                
                //contigency to if I'm loosing robots fast, keep making them
                if(rc.getRobotCount() < 10 && robotsCreated > numRobotsToMake)
                {
                	robotsCreated = 0;
                }
                
                if(rc.isBuildReady() && robotsCreated <= numRobotsToMake)
                {

                    RobotType robotToMake = robotTypeToMake(lastRobotCreated);
                    
                    if(rc.canBuildRobot(robotToMake, dir))
                    {
	                	System.out.println("Building " + robotToMake);
	                    rc.buildRobot(robotToMake, dir);
	                    robotsCreated++;
	                    lastRobotCreated = robotToMake;
	                    if(robotToMake == RobotType.SOLDIER)
	                    {
	                    	solidersCreated++;
	                    }
	                    else if(robotToMake == RobotType.LUMBERJACK)
	                    {
	                    	lumberjacksCreated++;
	                    }
	                    builtSomething=true;
                    }
	             
                }
                
                if(!builtSomething)
                {
                	//find tree
                	TreeInfo[] trees = rc.senseNearbyTrees(-1f);
                	for(TreeInfo tree: trees)
                	{
                			
                			if(tree.getTeam() == friendly || tree.getTeam() == Team.NEUTRAL)
                			{
                				
                				MapLocation loc = tree.getLocation();
                				if(rc.canInteractWithTree(loc))
                				{
                					//interact with tree
                					if(rc.canWater(tree.getID()) && (tree.getHealth()/tree.getMaxHealth()) < .85 && tree.getTeam() != Team.NEUTRAL)
                					{
                						System.out.println("Watering tree");
                						rc.water(loc);
                						break;
                					}
                					else if(rc.canShake(loc) && tree.containedBullets > 0)
                					{
                						System.out.println("Shaking tree");
                						rc.shake(tree.getID());
                						break;
                					}
                					else
                					{
                						Direction treeDir = randomDirection();
                						TreeInfo[] nearbyTrees = rc.senseNearbyTrees(-1);
                						if(rc.canPlantTree(treeDir) && nearbyTrees.length < 2)
                						{
                							System.out.println("Planting tree");
                							rc.plantTree(treeDir);
                							break;
                						}
                						else
                						{
                							tryMove(randomDirection());
                							break;
                						}
                					}
                				}
                				else
                				{
                					if(!johnnyAppleseed)
                					{
	                					MapLocation myLocation = rc.getLocation();
	                                	Direction toTree = myLocation.directionTo(loc);
	                                	tryMove(toTree);
                					}
                					else
                					{
                						tryMove(randomDirection());
                					}

                					break;
                				}
                			
                		}
                	}
                }

                // Move randomly
                tryMove(randomDirection());
               
                
                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Gardener Exception");
                e.printStackTrace();
            }
        }
    }

	private static RobotType robotTypeToMake(RobotType lastRobotCreated) {
		RobotType robotToMake = RobotType.SOLDIER;
		if(lastRobotCreated == null)
		{
			robotToMake = RobotType.SOLDIER;
		}
		else if(lastRobotCreated == RobotType.SOLDIER)
		{
			robotToMake = RobotType.LUMBERJACK;
		}
		else if(lastRobotCreated == RobotType.LUMBERJACK)
		{
			robotToMake = RobotType.SOLDIER;
		}
		return robotToMake;
	}

    static void runSoldier() throws GameActionException {
        System.out.println("I'm an soldier!");
        Team enemy = rc.getTeam().opponent();

        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {
                MapLocation myLocation = rc.getLocation();

                // See if there are any nearby enemy robots
                RobotInfo[] robots = rc.senseNearbyRobots(-1, enemy);

                // If there are some...
                if (robots.length > 0) {
                    // And we have enough bullets, and haven't attacked yet this turn...
                    if (rc.canFireSingleShot()) {
                        // ...Then fire a bullet in the direction of the enemy.
                        rc.fireSingleShot(rc.getLocation().directionTo(robots[0].location));
                    }
                }

                // Move randomly
                tryMove(randomDirection());

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Soldier Exception");
                e.printStackTrace();
            }
        }
    }

    static void runLumberjack() throws GameActionException {
        System.out.println("I'm a lumberjack!");
        Team enemy = rc.getTeam().opponent();

        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {

                // See if there are any enemy robots within striking range (distance 1 from lumberjack's radius)
                //RobotInfo[] robots = rc.senseNearbyRobots(RobotType.LUMBERJACK.bodyRadius+GameConstants.LUMBERJACK_STRIKE_RADIUS, enemy);

                /*if(robots.length > 0 && !rc.hasAttacked()) {
                    // Use strike() to hit all nearby robots!
                    rc.strike();
                } else {*/
                    // No close robots, so search for robots within sight radius
                    TreeInfo[] trees = rc.senseNearbyTrees();

                    // If there is a robot, move towards it
                    if(trees.length > 0 && !rc.hasAttacked()) {
                        MapLocation myLocation = rc.getLocation();
                        for(TreeInfo tree : trees)
                        {
                        	if(tree.getTeam() == enemy || tree.getTeam() == Team.NEUTRAL)
                        	{
                        		if(rc.canChop(tree.getLocation()))
                        		{
                        			if(tree.getContainedBullets() > 0)
                        			{
                        				rc.shake(tree.getID());
                        			}
                        			System.out.println("Chopping tree");
                        			rc.chop(tree.getLocation());
                        			break;
                        		}
                        		else
                        		{
                        			MapLocation treeLocation = tree.getLocation();
                                	Direction toTree = myLocation.directionTo(treeLocation);
                                	boolean success = tryMove(toTree);
                                	if(!success)
                                	{
                                		tryMove(randomDirection());
                                	}
                                	break;
                        		}
                        	}
                        }

                    }
                    else if (!rc.hasAttacked() && rc.senseNearbyRobots(RobotType.LUMBERJACK.bodyRadius+GameConstants.LUMBERJACK_STRIKE_RADIUS, enemy).length > 0)
                    {
                    	rc.strike();
                    }

               // }
                    tryMove(randomDirection());
                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Lumberjack Exception");
                e.printStackTrace();
            }
        }
    }

    /**
     * Returns a random Direction
     * @return a random Direction
     */
    static Direction randomDirection() {
        return new Direction((float)Math.random() * 2 * (float)Math.PI);
    }

    /**
     * Attempts to move in a given direction, while avoiding small obstacles directly in the path.
     *
     * @param dir The intended direction of movement
     * @return true if a move was performed
     * @throws GameActionException
     */
    static boolean tryMove(Direction dir) throws GameActionException 
    {
        try {
			return tryMove(dir,20,3);
		} catch (Exception e) {
			return false;
		}
    }

    /**
     * Attempts to move in a given direction, while avoiding small obstacles direction in the path.
     *
     * @param dir The intended direction of movement
     * @param degreeOffset Spacing between checked directions (degrees)
     * @param checksPerSide Number of extra directions checked on each side, if intended direction was unavailable
     * @return true if a move was performed
     * @throws GameActionException
     */
    static boolean tryMove(Direction dir, float degreeOffset, int checksPerSide) throws GameActionException {

        // First, try intended direction
        if (rc.canMove(dir)) {
            rc.move(dir);
            return true;
        }

        // Now try a bunch of similar angles
        boolean moved = false;
        int currentCheck = 1;

        while(currentCheck<=checksPerSide) {
            // Try the offset of the left side
            if(rc.canMove(dir.rotateLeftDegrees(degreeOffset*currentCheck))) {
                rc.move(dir.rotateLeftDegrees(degreeOffset*currentCheck));
                return true;
            }
            // Try the offset on the right side
            if(rc.canMove(dir.rotateRightDegrees(degreeOffset*currentCheck))) {
                rc.move(dir.rotateRightDegrees(degreeOffset*currentCheck));
                return true;
            }
            // No move performed, try slightly further
            currentCheck++;
        }

        // A move never happened, so return false.
        return false;
    }

    /**
     * A slightly more complicated example function, this returns true if the given bullet is on a collision
     * course with the current robot. Doesn't take into account objects between the bullet and this robot.
     *
     * @param bullet The bullet in question
     * @return True if the line of the bullet's path intersects with this robot's current position.
     */
    static boolean willCollideWithMe(BulletInfo bullet) {
        MapLocation myLocation = rc.getLocation();

        // Get relevant bullet information
        Direction propagationDirection = bullet.dir;
        MapLocation bulletLocation = bullet.location;

        // Calculate bullet relations to this robot
        Direction directionToRobot = bulletLocation.directionTo(myLocation);
        float distToRobot = bulletLocation.distanceTo(myLocation);
        float theta = propagationDirection.radiansBetween(directionToRobot);

        // If theta > 90 degrees, then the bullet is traveling away from us and we can break early
        if (Math.abs(theta) > Math.PI/2) {
            return false;
        }

        // distToRobot is our hypotenuse, theta is our angle, and we want to know this length of the opposite leg.
        // This is the distance of a line that goes from myLocation and intersects perpendicularly with propagationDirection.
        // This corresponds to the smallest radius circle centered at our location that would intersect with the
        // line that is the path of the bullet.
        float perpendicularDist = (float)Math.abs(distToRobot * Math.sin(theta)); // soh cah toa :)

        return (perpendicularDist <= rc.getType().bodyRadius);
    }
}
