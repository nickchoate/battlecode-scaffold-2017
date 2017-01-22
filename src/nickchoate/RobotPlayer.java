package nickchoate;

import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import battlecode.common.*;

public strictfp class RobotPlayer {
	static RobotController rc;

	/**
	 * run() is the method that is called when a robot is instantiated in the
	 * Battlecode world. If this method returns, the robot dies!
	 **/
	@SuppressWarnings("unused")
	public static void run(RobotController rc) throws GameActionException {

		// This is the RobotController object. You use it to perform actions
		// from this robot,
		// and to get information on its current status.
		RobotPlayer.rc = rc;

		// Here, we've separated the controls into a different method for each
		// RobotType.
		// You can add the missing ones or rewrite this into your own control
		// structure.
		switch (rc.getType()) {
		case ARCHON:
			runArchon();
			break;
		case GARDENER:
			runGardener();
			break;
		case SOLDIER:
			runFightingUnit(RobotType.SOLDIER);
			break;
		case LUMBERJACK:
			runLumberjack();
			break;
		case SCOUT:
			runFightingUnit(RobotType.SCOUT);
		case TANK:
			runFightingUnit(RobotType.TANK);
		}
	}

	public final static int ARCHON_CHANNEL_X = 0;
	public final static int ARCHON_CHANNEL_Y = 1;
	public final static int ARCHON_CHANNEL_NUM = 2;
	
	public final static double PERIMETER_RADIUS = 10;

	static void runArchon() throws GameActionException {
		System.out.println("I'm an archon!");
		int numGardenersCreated = 0;
		int gardenerLimit = 3;
		// The code you want your robot to perform every round should be in this
		// loop
		while (true) {

			// Try/catch blocks stop unhandled exceptions, which cause your
			// robot to explode
			try {

				if (rc.getRoundNum() % 500 == 0) {
					gardenerLimit = gardenerLimit + 5;
				}
				// Generate a random direction
				Direction dir = randomDirection();

				if (rc.canHireGardener(dir) && rc.getBuildCooldownTurns() == 0 && numGardenersCreated < gardenerLimit) {
					System.out.println("Build gardener");
					rc.hireGardener(dir);
					numGardenersCreated++;
				} else {
					float totalBullets = rc.getTeamBullets();
					if (totalBullets > 200) {
						rc.donate(100);
					}
				}

				// Broadcast archon's location for other robots on the team to
				// know
				MapLocation myLocation = rc.getLocation();
				rc.broadcast(ARCHON_CHANNEL_X, (int) myLocation.x);
				rc.broadcast(ARCHON_CHANNEL_Y, (int) myLocation.y);

				// Clock.yield() makes the robot wait until the next turn, then
				// it will perform this loop again
				Clock.yield();

			} catch (Exception e) {
				System.out.println("Archon Exception");
				e.printStackTrace();
			}
		}
	}

	static void runGardener() throws GameActionException {
		System.out.println("I'm a gardener!");
		RobotType lastRobotCreated = null; // 0 = solider 1 = lumberjack
		Team friendly = rc.getTeam();
		int randomUpperLimit = 7;
		Random rand = new Random();
		int robotsCreated = 0;
		int lumberjacksCreated = 0;
		int solidersCreated = 0;
		int numRobotsToMake = 2;
		int numTreesPlanted = 0;

		boolean johnnyAppleseed = true;
		if (rand.nextInt() % 2 == 0) {
			johnnyAppleseed = false;
		}

		// The code you want your robot to perform every round should be in this
		// loop
		while (true) {

			// Try/catch blocks stop unhandled exceptions, which cause your
			// robot to explode
			try {

				// Listen for home archon's location
				// int xPos = rc.readBroadcast(ARCHON_CHANNEL_X);
				// int yPos = rc.readBroadcast(ARCHON_CHANNEL_Y);
				// MapLocation archonLoc = new MapLocation(xPos, yPos);

				// Generate a random direction
				Direction randomDirection = randomDirection();

				// Randomly attempt to build a soldier or lumberjack in this
				// direction
				boolean builtSomething = false;

				// contigency to if I'm loosing robots fast, keep making them
				if (robotsCreated >= numRobotsToMake && rc.getRobotCount() < 10) {
					robotsCreated = 0;
				}

				if (rc.isBuildReady() && robotsCreated <= numRobotsToMake && !johnnyAppleseed) {

					RobotType robotToMake = robotTypeToMake(rand, lastRobotCreated);

					if (rc.canBuildRobot(robotToMake, randomDirection)) {
						System.out.println("Building " + robotToMake);
						rc.buildRobot(robotToMake, randomDirection);
						robotsCreated++;
						lastRobotCreated = robotToMake;
						if (robotToMake == RobotType.SOLDIER) {
							solidersCreated++;
						} else if (robotToMake == RobotType.LUMBERJACK) {
							lumberjacksCreated++;
						}
						builtSomething = true;
					}

				}

				TreeInfo[] trees = rc.senseNearbyTrees(-1f, friendly);
				// no friendly trees around, let's plant one!
				Direction treeDir = randomDirection();
				if (johnnyAppleseed && trees.length < 4 && rc.canPlantTree(treeDir)) {
					rc.plantTree(treeDir);
				} else // let's maintain the trees we have
				{
					boolean wateredTree = false;
					MapLocation lowHealthTree = null;
					boolean firstRun = true;
					float lowestHealth = 0;
					for (TreeInfo tree : trees) {
						if (firstRun) {
							lowestHealth = tree.getHealth();
							firstRun = false;
						}

						if (lowestHealth > tree.getHealth()) {
							lowestHealth = tree.getHealth();
							lowHealthTree = tree.getLocation();
						}
						MapLocation loc = tree.getLocation();
						if (rc.canInteractWithTree(loc)) {
							// interact with tree
							if (rc.canWater(tree.getID()) && (tree.getHealth() / tree.getMaxHealth()) < .9) {
								
								rc.water(loc);
								wateredTree = true;
								break;
							}
						}

					}

					if (!wateredTree && trees.length > 0) {
						MapLocation myLocation = rc.getLocation();
						Direction toTree = myLocation.directionTo(lowHealthTree);
						tryMove(toTree);
					}

					if (trees.length == 0) {
						tryMove(randomDirection);
					}

				}

				// Move randomly
				tryMove(randomDirection());

				// Clock.yield() makes the robot wait until the next turn, then
				// it will perform this loop again
				Clock.yield();

			} catch (Exception e) {
				System.out.println("Gardener Exception");
				e.printStackTrace();
			}
		}
	}

	private static RobotType robotTypeToMake(Random rand, RobotType lastRobotCreated) {
		RobotType robotToMake = RobotType.SOLDIER;
		if (lastRobotCreated == RobotType.LUMBERJACK || lastRobotCreated == null) {
			int mod = rand.nextInt() % 4;
			if (mod != 1) {
				robotToMake = RobotType.SOLDIER;
			} else {
				robotToMake = RobotType.SOLDIER;
			}
		} else if (lastRobotCreated == RobotType.SOLDIER || lastRobotCreated == RobotType.SCOUT) {
			robotToMake = RobotType.LUMBERJACK;
		}

		return robotToMake;
	}

	static void runFightingUnit(RobotType robotType) throws GameActionException {
		System.out.println("I'm an " + robotType + "!");
		Team enemy = rc.getTeam().opponent();
		Team friendly = rc.getTeam();
		MapLocation archonLoc = null;
		MapLocation finalPosition = null;

		// The code you want your robot to perform every round should be in this
		// loop
		while (true) {

			// Try/catch blocks stop unhandled exceptions, which cause your
			// robot to explode
			try {

				// form perimeter
				if (robotType == RobotType.SOLDIER && finalPosition == null) {
					if (archonLoc == null) {
						RobotInfo[] robots = rc.senseNearbyRobots(RobotType.SOLDIER.sensorRadius, friendly);
						for (RobotInfo robot : robots) {
							if (robot.getType() == RobotType.ARCHON) {
								archonLoc = robot.getLocation();
								break;
							}
						}
						
						if(archonLoc == null)
						{
							int xPos = rc.readBroadcast(ARCHON_CHANNEL_X);
							int yPos = rc.readBroadcast(ARCHON_CHANNEL_Y);
							archonLoc = new MapLocation(xPos, yPos);
							
						}
					}
					
					if(archonLoc != null)
					{
						if(finalPosition == null)
						{
							double totalRadius = RobotType.ARCHON.bodyRadius + PERIMETER_RADIUS;
							double startingRadius = 0;
							double radians = 0;
							do
							{
								double newX = totalRadius * Math.cos(radians) + archonLoc.x;
								double newY = totalRadius * Math.sin(radians) + archonLoc.y;
								finalPosition = new MapLocation(new Double(newX).floatValue(),new Double(newY).floatValue());
								radians = radians + .261799;
							}
							while(!rc.isLocationOccupied(finalPosition) && radians < 2 * Math.PI);
							
							if(rc.isLocationOccupied(finalPosition))
							{
								finalPosition = rc.getLocation();
							}
						}
						
					}
					
					if(rc.canMove(finalPosition))
					{
						rc.move(finalPosition);
					}
					else 
					{
						tryMove(rc.getLocation().directionTo(finalPosition));
					}
					
				}

				// See if there are any nearby enemy robots
				RobotInfo[] robots = rc.senseNearbyRobots(-1, enemy);

				// If there are some...
				if (robots.length > 0) {
					// And we have enough bullets, and haven't attacked yet this
					// turn...
					if (rc.canFireSingleShot()) {
						// ...Then fire a bullet in the direction of the enemy.
						rc.fireSingleShot(rc.getLocation().directionTo(robots[0].location));
					}
				}

				// Move randomly
				//tryMove(randomDirection());

				// Clock.yield() makes the robot wait until the next turn, then
				// it will perform this loop again
				Clock.yield();

			} catch (Exception e) {
				System.out.println(robotType + " Exception");
				e.printStackTrace();
			}
		}
	}

	static void runLumberjack() throws GameActionException {
		System.out.println("I'm a lumberjack!");
		Team enemy = rc.getTeam().opponent();

		// The code you want your robot to perform every round should be in this
		// loop
		while (true) {

			// Try/catch blocks stop unhandled exceptions, which cause your
			// robot to explode
			try {

				// See if there are any enemy robots within striking range
				// (distance 1 from lumberjack's radius)
				// RobotInfo[] robots =
				// rc.senseNearbyRobots(RobotType.LUMBERJACK.bodyRadius+GameConstants.LUMBERJACK_STRIKE_RADIUS,
				// enemy);

				/*
				 * if(robots.length > 0 && !rc.hasAttacked()) { // Use strike()
				 * to hit all nearby robots! rc.strike(); } else {
				 */
				// No close robots, so search for robots within sight radius
				TreeInfo[] trees = rc.senseNearbyTrees();

				// If there is a tree, move towards it
				if (trees.length > 0 && !rc.hasAttacked()) {
					MapLocation myLocation = rc.getLocation();
					for (TreeInfo tree : trees) {
						if (tree.getTeam() == enemy || tree.getTeam() == Team.NEUTRAL) {
							if (rc.canChop(tree.getLocation())) {
								if (tree.getContainedBullets() > 0) {
									rc.shake(tree.getID());
								}
								rc.chop(tree.getLocation());
								break;
							} else {
								MapLocation treeLocation = tree.getLocation();
								Direction toTree = myLocation.directionTo(treeLocation);
								boolean success = tryMove(toTree);
								/*
								 * if(!success) { tryMove(randomDirection()); }
								 */
								break;
							}
						}
					}

				} else {
					tryMove(randomDirection());
				}

				if (!rc.hasAttacked() && rc.senseNearbyRobots(
						RobotType.LUMBERJACK.bodyRadius + GameConstants.LUMBERJACK_STRIKE_RADIUS, enemy).length > 0) {
					rc.strike();
				}

				// }
				tryMove(randomDirection());
				// Clock.yield() makes the robot wait until the next turn, then
				// it will perform this loop again
				Clock.yield();

			} catch (Exception e) {
				System.out.println("Lumberjack Exception");
				e.printStackTrace();
			}
		}
	}

	/**
	 * Returns a random Direction
	 * 
	 * @return a random Direction
	 */
	static Direction randomDirection() {
		return new Direction((float) Math.random() * 2 * (float) Math.PI);
	}

	/**
	 * Attempts to move in a given direction, while avoiding small obstacles
	 * directly in the path.
	 *
	 * @param dir
	 *            The intended direction of movement
	 * @return true if a move was performed
	 * @throws GameActionException
	 */
	static boolean tryMove(Direction dir) throws GameActionException {
		try {
			return tryMove(dir, 20, 3);
		} catch (Exception e) {
			return false;
		}
	}

	/**
	 * Attempts to move in a given direction, while avoiding small obstacles
	 * direction in the path.
	 *
	 * @param dir
	 *            The intended direction of movement
	 * @param degreeOffset
	 *            Spacing between checked directions (degrees)
	 * @param checksPerSide
	 *            Number of extra directions checked on each side, if intended
	 *            direction was unavailable
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

		while (currentCheck <= checksPerSide) {
			// Try the offset of the left side
			if (rc.canMove(dir.rotateLeftDegrees(degreeOffset * currentCheck))) {
				rc.move(dir.rotateLeftDegrees(degreeOffset * currentCheck));
				return true;
			}
			// Try the offset on the right side
			if (rc.canMove(dir.rotateRightDegrees(degreeOffset * currentCheck))) {
				rc.move(dir.rotateRightDegrees(degreeOffset * currentCheck));
				return true;
			}
			// No move performed, try slightly further
			currentCheck++;
		}

		// A move never happened, so return false.
		return false;
	}

	/**
	 * A slightly more complicated example function, this returns true if the
	 * given bullet is on a collision course with the current robot. Doesn't
	 * take into account objects between the bullet and this robot.
	 *
	 * @param bullet
	 *            The bullet in question
	 * @return True if the line of the bullet's path intersects with this
	 *         robot's current position.
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

		// If theta > 90 degrees, then the bullet is traveling away from us and
		// we can break early
		if (Math.abs(theta) > Math.PI / 2) {
			return false;
		}

		// distToRobot is our hypotenuse, theta is our angle, and we want to
		// know this length of the opposite leg.
		// This is the distance of a line that goes from myLocation and
		// intersects perpendicularly with propagationDirection.
		// This corresponds to the smallest radius circle centered at our
		// location that would intersect with the
		// line that is the path of the bullet.
		float perpendicularDist = (float) Math.abs(distToRobot * Math.sin(theta)); // soh
																					// cah
																					// toa
																					// :)

		return (perpendicularDist <= rc.getType().bodyRadius);
	}
}
