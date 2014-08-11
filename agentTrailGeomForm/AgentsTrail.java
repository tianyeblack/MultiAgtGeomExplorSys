package agentTrailGeomForm;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;

import toxi.geom.Vec3D;
import toxi.volume.VolumetricSpace;
import toxi.volume.VolumetricSpaceArray;

public class AgentsTrail {
	private static AgentsTrail _instance = null;
	
	//ratio between agents of type b and c 
	public static final double agentRatio = 0.8;
	// Dimensions of the space we are working
	public int DIMX = 1000, DIMY = 1000, DIMZ =200;
	//Affects the resolution and the FrameRate
	public int GRIDX = 250, GRIDY = 250, GRIDZ = 40;
	public VolumetricSpace volumeA;
	public VolumetricSpace volumeB;
	public VolumetricSpace volumeC;
	
	//level where agents of type 1 are being created 
	public int creationLevel = 10;
	ArrayList<Agent> agents;
	ArrayList<AgentLine> connections;
	public Vec3D[] geo_starts;
	public Vec3D[] outlines;
	public Vec3D[] twod_plane;
	public float[] geo_scores;
	public float[] out_scores;
	public float[] twod_scores;

	protected AgentsTrail() {
		agents = new ArrayList<Agent>();
		connections = new ArrayList<AgentLine>();
	}
	
	public static AgentsTrail getInstance() {
		if (null == _instance) _instance = new AgentsTrail();
		return _instance;
	}

	public ArrayList<Agent> getAgents() {
		return agents;
	}
	
	public ArrayList<Vec3D> getLocs() {
		ArrayList<Vec3D> locs = new ArrayList<Vec3D>();
		for (Agent a : agents) locs.add(a.loc);
		return locs;
	}
	
	public ArrayList<Vec3D> getStarts() {
		ArrayList<Vec3D> all_starts = new ArrayList<Vec3D>();
		for (Agent a : agents) all_starts.add(a.start);
		return all_starts;
	}
	
	public ArrayList<AgentLine> getConnections() {
		return connections;
	}

	public void setDIMAndGRID(int _DIMX, int _DIMY, int _DIMZ, int ratio) {
		DIMX = _DIMX;
		GRIDX = DIMX / ratio;
		DIMY = _DIMY;
		GRIDY = DIMY / ratio;
		DIMZ = _DIMZ;
		GRIDZ = DIMZ / ratio;
		Vec3D SCALE = new Vec3D(DIMX, DIMY, DIMZ);
		volumeA = volumeB = volumeC = null;
		volumeA = new VolumetricSpaceArray(SCALE, GRIDX, GRIDY, GRIDZ);
		volumeB = new VolumetricSpaceArray(SCALE, GRIDX, GRIDY, GRIDZ);
		volumeC = new VolumetricSpaceArray(SCALE, GRIDX, GRIDY, GRIDZ);
		Agent.a = DIMX / 2;
		Agent.b = DIMY / 2;
		Agent.c = DIMZ / 2;
	}
	
	public void addAgents(int pop, float[] scores, Vec3D[] starts) {
		int size = agents.size();
		for (int i = 0; i < pop; i++) {
			agents.add(new Agent(i + size, starts[i], scores[i], "", agents, pop + size));
		}
	}

	public void assignAgentsType(int type) {
		if (type == 1) {
			for (Agent a : agents) {
				if (a.start.z() < creationLevel) a.setType("a");
			}
		} else if (type == 2) {
			for (Agent a : agents) {
				if (a.start.z() >= creationLevel) {
					if (Math.random() >= agentRatio) a.setType("b");
					else a.setType("c");
				}
			}
		}
	}
	
	public void exportAgents(String filename) {
		try {
			PrintWriter pw = new PrintWriter(filename);
			for (Agent a : agents) {
				pw.println(a.getLoc());
			}
			pw.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return;
		}
	}
	
	public void runAgents(int iteration) {
		for (Agent a : agents) a.update();
		for (int i = 0; i < agents.size(); i++) {
			Agent a = agents.get(i);
			a.run(iteration);
		}
//		for (int i = 0; i < agents.size(); i++) {
//			Agent a = agents.get(i);
//			a.agentConnection(10, connections, iteration);
//		}
	}
	
	public void switchAgents() {
		for (Agent a : agents) a.runToggle = !a.runToggle; 
	}
	
	public void exportText(int frameCount) {
		PrintWriter output;
		try {
			output = new PrintWriter("output/agentPositions" + frameCount + ".txt");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return;
		}
		for (Agent a : agents) {
			output.println(a.loc.x + "," + a.loc.y + "," + a.loc.z);
		}
		output.flush();
		output.close();
	}
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
	}
}
