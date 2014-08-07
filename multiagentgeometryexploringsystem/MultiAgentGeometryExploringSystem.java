package multiagentgeometryexploringsystem;

import java.util.ArrayList;

import agentTrailGeomForm.Agent;
import agentTrailGeomForm.AgentLine;
import agentTrailGeomForm.AgentsTrail;
import controlP5.ControlP5;
import peasy.PeasyCam;
import processing.core.PApplet;
import toxi.geom.Vec3D;
import toxi.geom.mesh.TriangleMesh;
import toxi.processing.ToxiclibsSupport;
import toxi.volume.ArrayIsoSurface;
import toxi.volume.BoxBrush;
import toxi.volume.IsoSurface;
import toxi.volume.RoundBrush;
import toxi.volume.VolumetricBrush;
import toxi.volume.VolumetricSpaceArray;


@SuppressWarnings("serial")
public class MultiAgentGeometryExploringSystem extends PApplet {

	PeasyCam cam;
	VolumetricBrush brushA;
	VolumetricBrush brushB;
	VolumetricBrush brushC;
	VolumetricSpaceArray volumeA;
	VolumetricSpaceArray volumeB;
	VolumetricSpaceArray volumeC;
	IsoSurface surfaceA;
	IsoSurface surfaceB;
	IsoSurface surfaceC;
	TriangleMesh meshA = new TriangleMesh("meshA");
	TriangleMesh meshB = new TriangleMesh("meshB");
	TriangleMesh meshC = new TriangleMesh("meshC");
	ToxiclibsSupport gfx;
	ControlP5 ui;
	int movieCounter ;
	boolean videoRecord = false;
	int counter = 0;
	int pop=50;
	boolean runToggle = true;
	boolean capture = false;
	boolean record = false;
	boolean strok = false;
	boolean compute = false;
	boolean trail = false;

	//Box size
	int bX;
	int bY;
	int bZ;
	//Affects the mesh	
	float ISO = 0.5f;
	//Affects the resolution and the FrameRate
	int GRID = 200;
	int GRIDX;
	int GRIDY;
	int GRIDZ;
	// Dimensions of the space we are working
	int DIM = 3000;

	int DIMX, DIMY, DIMZ;
	int ratio = 4;
	float isoBrushSize = ratio;
	float isoBrushDensity = 1f;


	Vec3D SCALE = new Vec3D(DIM, DIM, DIM);

	AgentsTrail agt;
	Vec3D[] starts;
	Vec3D[] outlines;
	float[] scores;

	public void setup() {
		size(1200, 800, OPENGL);
		smooth();
		cam = new PeasyCam(this, 600);
		//		cam.lookAt(800, -200, 800);

		getGeometry("src/data/catenary_mesh_relaxed_03a.txt");
	//	getBoundary("src/data/catenary_mesh_relaxed_04_outlines.txt");
		
		
		agt.assignAgentsType(1);
		agt.assignAgentsType(2);
		SCALE = new Vec3D(bX, bY, bZ);
		GRIDX = bX / ratio;
		GRIDY = bY / ratio;
		GRIDZ = bZ / ratio;
		volumeA = new VolumetricSpaceArray(SCALE, GRIDX, GRIDY, GRIDZ);
		volumeB = new VolumetricSpaceArray(SCALE, GRIDX, GRIDY, GRIDZ);
		volumeC = new VolumetricSpaceArray(SCALE, GRIDX, GRIDY, GRIDZ);
		surfaceA = new ArrayIsoSurface(volumeA);
		surfaceB = new ArrayIsoSurface(volumeB);
		surfaceC = new ArrayIsoSurface(volumeC);
		brushA = new RoundBrush(volumeA, isoBrushSize);
		brushB = new RoundBrush(volumeB, 6f);
		brushC = new RoundBrush(volumeC, 4f);

		gfx = new ToxiclibsSupport(this);
		ui = new ControlP5(this);
		ui.setAutoDraw(false);
			

		ui.addSlider("ISO",0,1,ISO,20,20,300,14);
		//ui.addButton ("Isosruface",20,20,300,14));
	}

	@SuppressWarnings("deprecation")
	public void draw() {
		if (record) beginRaw(PDF, "msa_catenary_Srf_trails"+ frameCount+".pdf") ;
		background(255);
		lights();
		
		agt.runAgents(frameCount);
		if (trail == true) {
			displayTrails();
			displayConnections();
		} else {
			displayLocs();
		}

		// A bounding box for a better view
		stroke(0, 0, 192);
		strokeWeight(.5f);
		noFill();
		box(bX, bY, bZ);
		displayStarts();
		displayLocs();
		//displayOutline();

		if (frameCount % 5 == 0 && compute) {
			drawTrails();
			surfaceA.reset();
			surfaceA.computeSurfaceMesh(meshA, ISO);

			surfaceB.reset();
			surfaceB.computeSurfaceMesh(meshB, ISO);

			surfaceC.reset();
			surfaceC.computeSurfaceMesh(meshC, ISO);
		}

		if (strok) {
			stroke(0.1f);
		} else {
			noStroke();
			fill(0, 0, 255);			
			gfx.mesh(meshA, true);

			noStroke();
			fill(0, 255, 0);	
			gfx.mesh(meshB, true);

			noStroke();
			fill(255, 0, 0);	
			gfx.mesh(meshC, true);
		}

		if (frameCount == 1 || (frameCount % 20 == 0 && frameCount < 1000)) {
			agt.exportText(frameCount);
		}
		if (capture) saveFrame("MSA_agent_cat_Trails" + frameCount + ".png");
		if (record) {
			endRaw();
			record = false;
		}

		recording();
		if (ui.window(this).isMouseOver()) cam.setActive(false);
		else cam.setActive(true);
		gui();


	}

	void gui() {
		hint(DISABLE_DEPTH_TEST);
		cam.beginHUD();
		ui.draw();
		cam.endHUD();
		hint(ENABLE_DEPTH_TEST);
	}

	void getGeometry(String filename) {
		float x_min, y_min, z_min, x_max, y_max, z_max;
		x_min = Float.MAX_VALUE;
		y_min = Float.MAX_VALUE;
		z_min = Float.MAX_VALUE;
		x_max = Float.MIN_VALUE;
		y_max = Float.MIN_VALUE;
		z_max = Float.MIN_VALUE;
		String[] lines = loadStrings(filename);
		println("There are " + lines.length + " lines in the elevation point file..");
		scores = new float[lines.length];
		starts = new Vec3D[lines.length];
		for (int i = 0; i < lines.length; i++) {
			String[] parts = split(lines[i], "}");
			String[] coordinates = split(parts[0].substring(1), ", ");
			float x_cur = Float.parseFloat(coordinates[0]);
			float y_cur = Float.parseFloat(coordinates[1]);
			float z_cur = Float.parseFloat(coordinates[2]);
			if (x_cur < x_min) x_min = x_cur;
			else if (x_cur > x_max) x_max = x_cur;
			if (y_cur < y_min) y_min = y_cur;
			else if (y_cur > y_max) y_max = y_cur;
			if (z_cur < z_min) z_min = z_cur;
			else if (z_cur > z_max) z_max = z_cur;
			starts[i] = new Vec3D(x_cur, y_cur, z_cur);
			scores[i] = Float.parseFloat(parts[1]);
		}
		agt = new AgentsTrail(lines.length);
		Vec3D trans = new Vec3D(x_min, y_min, z_min);
		System.out.printf("%f, %f, %f\n", x_max - x_min, y_max - y_min, z_max - z_min);
		bX = (int) (Math.ceil((x_max - x_min) / ratio) * ratio);
		bY = (int) (Math.ceil((y_max - y_min) / ratio) * ratio);
		bZ = (int) (Math.ceil((z_max - z_min) / ratio) * ratio);
		System.out.printf("%d, %d, %d\n", bX, bY, bZ);
		DIMX = bX;
		DIMY = bY;
		DIMZ = bZ;
		trans.addSelf(new Vec3D(DIMX / 2, DIMY / 2, DIMZ / 2));
		for (Vec3D v : starts) v.subSelf(trans);
		agt.createAgents(lines.length, scores, starts);
	}

	
	
	void getBoundary(String filename){
		float x_min, y_min, z_min, x_max, y_max, z_max;
		x_min = Float.MAX_VALUE;
		y_min = Float.MAX_VALUE;
		z_min = Float.MAX_VALUE;
		x_max = Float.MIN_VALUE;
		y_max = Float.MIN_VALUE;
		z_max = Float.MIN_VALUE;
		String[] lines=loadStrings (filename);
		outlines = new Vec3D[lines.length];
		for (int i = 0; i < lines.length; i++) {
			String[] coordinates = split(lines[i].substring(1, lines[i].length() - 1), ", ");
			float x_cur = Float.parseFloat(coordinates[0]);
			float y_cur = Float.parseFloat(coordinates[1]);
			float z_cur = Float.parseFloat(coordinates[2]);
			if (x_cur < x_min) x_min = x_cur;
			else if (x_cur > x_max) x_max = x_cur;
			if (y_cur < y_min) y_min = y_cur;
			else if (y_cur > y_max) y_max = y_cur;
			if (z_cur < z_min) z_min = z_cur;
			else if (z_cur > z_max) z_max = z_cur;
			outlines[i]= new Vec3D (x_cur, y_cur, z_cur);
			println (outlines[i]); 
		}
		
		agt = new AgentsTrail(lines.length);
		Vec3D trans = new Vec3D(x_min, y_min, z_min);
		System.out.printf("%f, %f, %f\n", x_max - x_min, y_max - y_min, z_max - z_min);
		bX = (int) (Math.ceil((x_max - x_min) / ratio) * ratio);
		bY = (int) (Math.ceil((y_max - y_min) / ratio) * ratio);
		bZ = (int) (Math.ceil((z_max - z_min) / ratio) * ratio);
		System.out.printf("%d, %d, %d\n", bX, bY, bZ);
		DIMX = bX;
		DIMY = bY;
		DIMZ = bZ;
		trans.addSelf(new Vec3D(DIMX / 2, DIMY / 2, DIMZ / 2));
		for (Vec3D v : outlines) v.subSelf(trans);
		agt.createAgents(lines.length, scores, outlines);
		
		
	}
	
	
	
	
	void displayOutline() {
		for (Vec3D v : outlines) {
			stroke(250,0,250);
			strokeWeight(4);
			noFill();
			point(v.x, v.y, v.z);
		}
		
	}
	
	
	void displayStarts() {
		for (Vec3D v : starts) {
			stroke(150);
			strokeWeight(2f);
			noFill();
			point(v.x, v.y, v.z);
			pushMatrix();
			translate(v.x, v.y, v.z);
			//box(20);
			popMatrix();
		}
	}

	void displayLocs() {
		strokeWeight(4);// no stroke for the shape.
		for (Agent a : agt.getAgents()) {
			String agtType = a.getType();
			Vec3D loc = a.getLoc();
			if (agtType.equals("a")) {
				stroke(0, 0, 250);// fill color, a dark blue.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
			if (agtType.equals("b")) {
				stroke(0, 250, 0);// fill color, a dark green.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
			if (agtType.equals("c")) {
				stroke(250, 0, 0);// fill color, a dark red.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
		}
	}

	void displayTrails() {
		strokeWeight(2);// no stroke for the shape.
		for (Agent a : agt.getAgents()) {
			String agtType = a.getType();
			ArrayList<Vec3D> trail = a.getTrail();
			if (agtType.equals("a")) {
				stroke(0, 0, 200);// fill color, a dark blue.
				for (Vec3D v : trail) {
					point(v.x, v.y, v.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
				}
			}
			if (agtType.equals("b")) {
				stroke(0, 200, 0);// fill color, a dark green.
				for (Vec3D v : trail) {
					point(v.x, v.y, v.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
				}
			}
			if (agtType.equals("c")) {
				stroke(200, 0, 0);// fill color, a dark red.
				for (Vec3D v : trail) {
					point(v.x, v.y, v.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
				}
			}
		}
	}

	void displayConnections() {
		ArrayList<AgentLine> connections = agt.getConnections();
		for (AgentLine l : connections) {
			stroke(64);
			strokeWeight(1.0f);
			Vec3D pta = l.getPta();
			Vec3D ptb = l.getPtb();
			line(pta.x, pta.y, pta.z, ptb.x, ptb.y, ptb.z);
		}
	}
	
	void drawLines(){
		for (Agent a : agt.getAgents()) {
			
		}
	}


	void drawTrails() {
		for (Agent a : agt.getAgents()) {
			if (!a.getType().equals("")) {
				VolumetricBrush brush;
				if(a.getType().equals("a")){
					brush = brushA;
					stroke(0, 255, 0);
					strokeWeight(2.0f);
				} else if(a.getType().equals("b")){
					brush = brushB;
					stroke(255, 255, 0);
					strokeWeight(1.0f);
				} else if(a.getType().equals("c")) {
					brush = brushC;
					stroke(0, 0, 255);
					strokeWeight(1.0f);
				} else brush = brushA;


				for (Vec3D v : a.getTrail()) {
					point(v.x, v.y, v.z);
					brush.drawAtAbsolutePos(v, isoBrushDensity);
				}
			}
		}
	}

	public void keyPressed() {

		////Video
		if (key == 'r') {
			videoRecord = ! videoRecord;  // start/stop the movie (one press start, second press stop)
			if (videoRecord) {
				println("MOVIE STARTED!");
				movieCounter = 0;
			}
			if (!videoRecord) { 
				println("MOVIE STOPPED!");
			}
		}


		if (key=='s'){
			meshA.saveAsSTL(sketchPath(meshA.name + frameCount+ counter + ".stl"));
			meshB.saveAsSTL(sketchPath(meshB.name + frameCount+ counter + ".stl"));
			meshC.saveAsSTL(sketchPath(meshC.name + frameCount+ counter + ".stl"));
			counter = counter + 1;
			println ("Saved Successfull");
		}
		if (key=='e' || key== 'E') {
			// saveFrame("/output/seq-####.jpg");
			saveFrame("MSA_agent_cat_Trails" + frameCount + ".png");
			println("saved a frame");
		}
		if (key=='r') {
			capture= !capture;
		}
		if (key == 'n') {
			runToggle = !runToggle;
		}
		if (key == 'p') {
			record= !record;
		}
		if (key == 'k') {
			strok = !strok;
		}
		if (key == 'c' ||key == 'C') {
			compute = !compute;
		}
		if (key == 't') trail = !trail;
	}




	void recording(){
		if(videoRecord==true){

			String saveName = "Agents" + "__" + nf(movieCounter, 4) + ".png";
			saveFrame(saveName);
			println(saveName);
			movieCounter+=2;
		}
	}


	public static void main(String _args[]) {
		PApplet.main(new String[] { multiagentgeometryexploringsystem.MultiAgentGeometryExploringSystem.class.getName() });
	}
}
