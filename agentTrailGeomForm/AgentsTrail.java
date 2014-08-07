package agentTrailGeomForm;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;

import toxi.geom.Vec3D;

public class AgentsTrail {
	//level where agents of type 1 are being created 
	static int creationLevel = 50;
	ArrayList<Agent> agents;

	public AgentsTrail(int size) {
		agents = new ArrayList<Agent>(size);
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
		ArrayList<Vec3D> starts = new ArrayList<Vec3D>();
		for (Agent a : agents) starts.add(a.start);
		return starts;
	}

	public void createAgents(int pop, float[] scores, Vec3D[] starts) {
		for (int i = 0; i < pop; i++) {
			agents.add(new Agent(i, starts[i], scores[i], "", agents));
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
					if (Math.random() >= 0.5) a.setType("b");
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
