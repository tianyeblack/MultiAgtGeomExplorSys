package agentTrailGeomForm;

import java.util.ArrayList;

import toxi.geom.Vec3D;

public class Agent {
	static final float maxvel = 2;
	static final float maxForce = 10;
	static final int every = 5;
	static final int trailNum = 200;

	static final float alignment = 0.03f;
	static final float cohesion = 0.9f;
	static final float separation = 0.0f;

	static final float faceAttraction = 0.2f;
	static final float onSrfMotion = 0.05f;
	static final float trailFollow = 0.05f;

	static final int fovAlign = 100;
	static final int fovCoh = 200;
	static final int fovSep = 200;
	static final int fovScore = 300;
	static final int overkillDist = 5;

	static final float thresh = 50f;
	static final float agentBoxSize = 10f;

	int ID;									// identification in container
	Vec3D loc;								// current location
	Vec3D vel;								// current velocity
	Vec3D acc;								// current acceleration
	public Vec3D start;						// starting point
	ArrayList<Vec3D> trail;					// trail of this agent
	boolean runToggle;						// moving or stopping
	float score;							// property 1
	String agentType;						// meaningful name for agents type
	ArrayList<Agent> agents;
	float[] dist_to_agent;
	float[] dist_to_start;

	Agent(int _ID, Vec3D _start, float _score, String _agentType, ArrayList<Agent> _agents, int _pop) {
		ID = _ID;
		start = new Vec3D(_start.x(), _start.y(), _start.z());
		loc = new Vec3D(_start.x(), _start.y(), _start.z());
		acc = new Vec3D();
		runToggle = true;
		score = _score;
		agentType = _agentType;
		if (agentType.equals("a")) {
			vel = new Vec3D(0, 0, (float)(2 * Math.random() - 1));
		} else if (agentType.equals("b")) {
			vel = new Vec3D((float)(8 * Math.random() - 4), (float)(8 * Math.random() - 4), 0);
		} else if (agentType.equals("c")) {
			vel = new Vec3D((float)(8 * Math.random() - 4), (float)(8 * Math.random() - 4), 0);
		} else {
			vel = new Vec3D();
		}
		agents = _agents;
		trail = new ArrayList<Vec3D>();
		dist_to_agent = new float[_pop];
		dist_to_start = new float[_pop];
	}

	public ArrayList<Vec3D> getTrail() {
		return trail;
	}

	public Vec3D getLoc() {
		return loc.copy();
	}

	public void setType(String agtType) {
		agentType = agtType;
	}

	public String getType() {
		return agentType;
	}

	public void run(int iteration) {
		if (runToggle == true) {
			for (int i = 0; i < agents.size(); i++) {
				Agent a = agents.get(i);
				dist_to_start[i] = loc.distanceTo(a.start);
				dist_to_agent[i] = loc.distanceTo(a.loc);
			}
			//	if (agentType.equals("a") || agentType.equals("b")) {
			flock();
			//	}
			attractFaces(faceAttraction);
			if (agentType.equals("c")) {
				moveOnSrf(onSrfMotion);
				followTrails(trailFollow);
			}
			if (iteration % every == 0) {
				dropTrail(trailNum);
			}
		}
	}

	private void flock() {
		separation(separation);
		cohesion(cohesion);
		alignment(alignment);
	}

	public void agentConnection(float view, ArrayList<AgentLine> connections, int iteration) {
		if (iteration % every == 0 && runToggle == true) {
			int count = 0;
			for (int i = 0; i < agents.size() && count <= 3; i++) {
				if (dist_to_agent[i] < view && dist_to_agent[i] > 4) {
					count++;
					Agent a = agents.get(i);
					connections.add(new AgentLine(this, a, 
							new Vec3D(loc.x(), loc.y(), loc.z()),
							new Vec3D(a.loc.x(), a.loc.y(), a.loc.z()),
							iteration / every, iteration / every));
				}
			}
		}
	}

	public void update() {
		if (runToggle == true) {
			vel.addSelf(acc);
			vel.limit(maxvel);
			loc.addSelf(vel);
			acc.clear();
		}
	}

	private void dropTrail(int limit) {
		trail.add(loc.copy());
	}

	private void moveOnSrf(float magnitude) {
		int highID = 0;
		float thisScore = score;

		for (int i = 0; i < agents.size(); i++) {
			Agent a = agents.get(i);
			if (dist_to_start[i] > 0 && dist_to_start[i] < fovScore) {
				if (a.score < thisScore) {
					highID = a.ID;
					thisScore = a.score;
					if (dist_to_start[i] < 40) runToggle = false;
				}
			}
		}
		Vec3D target = agents.get(highID).start;
		Vec3D steeringVector = steer(target, false);
		steeringVector.scaleSelf(magnitude);
		acc.addSelf(steeringVector);
	}

	private void attractFaces(float magnitude) {
		Vec3D sum = new Vec3D();
		int count = 0;
		for (int i = 0; i < agents.size(); i++) {
			Agent a = agents.get(i);
			if (dist_to_start[i] > 0 && dist_to_start[i] < 100) {
				if (dist_to_start[i] < 50) {
					sum.addSelf(a.start);
					count++;
				}
				Vec3D steeringVector = steer(a.start, false);
				steeringVector.normalizeTo(1 / dist_to_start[i]);
				steeringVector.scaleSelf(a.score);
				acc.addSelf(steeringVector);
			}
		}
		if (count > 0) sum.scaleSelf(1.0f / count);
		Vec3D steering = sum.sub(loc);
		steering.scaleSelf(magnitude);
		acc.addSelf(steering);
	}

	private void followTrails(float magnitude) {
		int cloAID = -1;
		int cloTID = -1;
		float cloDist = 1000;
		Vec3D closestTrail = new Vec3D();
		Vec3D closestTrailFWD = new Vec3D();
		Vec3D steering = new Vec3D();
		for (Agent a : agents) {
			if (a != this && a.trail.size() > 3) {
				for (Vec3D v : a.trail) {
					float distance = loc.distanceTo(v);
					if (distance < cloDist) {
						cloAID = a.ID;
						cloTID = a.trail.indexOf(v);
						cloDist = distance;
					}
				}
			}
		}

		if (trail.size() > 3 && cloAID != -1 && cloTID != -1) {
			Agent cloA = agents.get(cloAID);
			closestTrail.addSelf(cloA.trail.get(cloTID));

			if (cloTID < cloA.trail.size() - 1) closestTrailFWD.set(cloA.trail.get(cloTID + 1));
			else if (cloTID == cloA.trail.size() - 1) closestTrailFWD.set(cloA.trail.get(cloTID));

			Vec3D mid = getNormalPoint(loc, closestTrail, closestTrailFWD);
			float distance = loc.distanceTo(mid);
			if (distance < 50) {
				seek(mid, magnitude);
				Vec3D heading = closestTrailFWD.sub(closestTrail);
				steering.addSelf(heading);

				if (distance < overkillDist) runToggle = false;
			}
			steering.scaleSelf(magnitude);
			acc.addSelf(steering);
		}
	}

	private void alignment(float magnitude) {
		Vec3D steering = new Vec3D();
		// We are creating a new empty vector to be added to the agent's acceleration.
		// This will be translated into a change in direction based on the calculations.
		int count = 0;
		// we create a variable called count. It will tell us how many times we have run into certain kinds of agents.
		for (int i = 0; i < agents.size(); i++) { // Loop through all agents
			Agent a = agents.get(i);
			float distance = dist_to_agent[i]; // distance to other agents
			if (distance > 0 && distance < fovAlign) { 
				// if in range (0, fovAlign), add the other agent's velocity to this agent's steering
				steering.addSelf(a.vel);
				count++;
			}
		}
		if (count > 0) steering.scaleSelf(1.0f / count);
		// scale the steering according the number of agents we run into
		steering.scaleSelf(magnitude);
		// scale again according to magnitude
		acc.addSelf(steering);
		// steer our agent by changing the acceleration, this operation will make all the agents start to move toward the same direction over time
	}

	private void cohesion(float magnitude) {
		Vec3D sum = new Vec3D();
		int count = 0;
		for (int i = 0; i < agents.size(); i++) {
			Agent a = agents.get(i);
			float distance = dist_to_agent[i];
			if (distance > 0 && distance < fovCoh) {
				sum.addSelf(a.loc);
				count++;
			}
		}
		if (count > 0) sum.scaleSelf(1.0f / count);
		sum.subSelf(loc);
		sum.scaleSelf(magnitude);
		acc.addSelf(sum);
	}

	private void separation(float magnitude) {
		Vec3D steering = new Vec3D();
		int count = 0;
		for (int i = 0; i < agents.size(); i++) {
			Agent a = agents.get(i);
			float distance = dist_to_agent[i];
			if (distance > 0 && distance < fovSep) {
				Vec3D diff = loc.sub(a.loc);
				diff.normalizeTo(1.0f / distance);
				steering.addSelf(diff);
				count++;
			}
		}
		if (count > 0) steering.scaleSelf(1.0f / count);
		steering.scaleSelf(magnitude);
		acc.addSelf(steering);
	}

	private Vec3D getNormalPoint(Vec3D p, Vec3D a, Vec3D b) {
		Vec3D ap = p.sub(a);
		Vec3D ab = b.sub(a);
		ab.normalize();
		ab.scaleSelf(ap.dot(ab));
		return a.add(ab);
	}

	private void seek(Vec3D target, float factor) {
		acc.addSelf(steer(target, false).scaleSelf(factor));
	}

	private Vec3D steer(Vec3D target, boolean slowdown) {
		Vec3D steering = new Vec3D();
		Vec3D desired = target.sub(loc);
		float d = desired.magnitude();
		if (d > 0) {
			desired.normalize();
			if (slowdown == true && d < 100) desired.scaleSelf(maxvel * (d / 100));
			else desired.scaleSelf(maxvel);
			steering.set(desired.sub(vel).limit(maxForce));
		}
		return steering;
	}
}
