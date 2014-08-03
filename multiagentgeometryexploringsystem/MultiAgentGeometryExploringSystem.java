package multiagentgeometryexploringsystem;

import java.util.ArrayList;

import agentTrailGeomForm.Agent;
import agentTrailGeomForm.AgentsTrail;
import controlP5.ControlP5;
import peasy.PeasyCam;
import processing.core.PApplet;
import toxi.geom.Vec3D;
import toxi.geom.mesh.TriangleMesh;
import toxi.processing.ToxiclibsSupport;
import toxi.volume.ArrayIsoSurface;
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

	int counter = 0;
	int pop=50;
	boolean runToggle = true;
	boolean capture = false;
	boolean record = false;
	boolean strok = false;
	boolean compute = false;

	//Box size
	int bX = 3000;
	int bY = 2000;
	int bZ = 1000;
	//Affects the mesh	
	float ISO = 0.5f;
	//Affects the resolution and the FrameRate
	int GRID = 200;
	// Dimensions of the space we are working
	int DIM = 3000;
	static final float isoBrushSize = 0.8f;
	static final float isoBrushDensity = 0.5f;

	Vec3D SCALE = new Vec3D(DIM, DIM, DIM);

	AgentsTrail agt;
	Vec3D[] starts;
	float[] scores;

	public void setup() {
		size(1200, 800, OPENGL);
		smooth();
		cam = new PeasyCam(this, 600);
		cam.lookAt(800, -200, 800);

		volumeA = new VolumetricSpaceArray(SCALE, GRID, GRID, GRID);
		volumeB = new VolumetricSpaceArray(SCALE, GRID, GRID, GRID);
		volumeC = new VolumetricSpaceArray(SCALE, GRID, GRID, GRID);
		surfaceA = new ArrayIsoSurface(volumeA);
		surfaceB = new ArrayIsoSurface(volumeB);
		surfaceC = new ArrayIsoSurface(volumeC);
		brushA = new RoundBrush(volumeA, 3f);
		brushB = new RoundBrush(volumeB, 2f);
		brushC = new RoundBrush(volumeC, 4f);

		gfx = new ToxiclibsSupport(this);
		ui = new ControlP5(this);
		ui.setAutoDraw(false);

		getGeometry(starts, scores, "src/data/catenary_mesh_relaxed_03a.txt");
		agt.assignAgentsType(1);
		agt.assignAgentsType(2);
		ui.addSlider("ISO",0,1,ISO,20,20,300,14);
	}

	@SuppressWarnings("deprecation")
	public void draw() {
		if (record) beginRaw(PDF, "msa_catenary_Srf_trails"+ frameCount+".pdf") ;
		background(255);
		lights();
		//displayStarts();

		agt.runAgents(frameCount);
		displayLocs();

		// A bounding box for a better view
		stroke(0, 0, 192);
		strokeWeight(.5f);
		noFill();
		box(bX, bY, bZ);

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
			fill(255, 0, 0);	
			gfx.mesh(meshB, true);

			noStroke();
			fill(0, 255, 0);	
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

	void getGeometry(Vec3D[] starts, float[] scores, String filename) {
		String[] lines = loadStrings(filename);
		println("There are " + lines.length + " lines in the elevation point file..");
		scores = new float[lines.length];
		starts = new Vec3D[lines.length];
		for (int i = 0; i < lines.length; i++) {
			String[] parts = split(lines[i], "}");
			String[] coordinates = split(parts[0].substring(1), ", ");
			starts[i] = new Vec3D(Float.parseFloat(coordinates[0]), Float.parseFloat(coordinates[1]), Float.parseFloat(coordinates[2]));
			scores[i] = Float.parseFloat(parts[1]);
		}
		agt = new AgentsTrail(lines.length);
		agt.createAgents(lines.length, scores, starts);
	}

	void displayStarts() {
		for (Vec3D v : starts) {
			stroke(100,0,0);
			strokeWeight(2f);
			noFill();
			point(v.x, v.y, v.z);
			pushMatrix();
			translate(v.x, v.y, v.z);
			//parent.box(20);
			popMatrix();
		}
	}

	void displayLocs() {
		for (Agent a : agt.getAgents()) {
			String agtType = a.getType();
			Vec3D loc = a.getLoc();
			if (agtType.equals("a")) {
				stroke(0, 250, 0);// fill color, a dark green.
				strokeWeight(4);// no stroke for the shape.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
			if (agtType.equals("b")) {
				stroke(0,0,250);// fill color, a dark green.
				strokeWeight(4);// no stroke for the shape.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
			if (agtType.equals("c")) {
				stroke(255, 255, 0);// fill color, a dark green.
				strokeWeight(4);// no stroke for the shape.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
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
					brush.setSize(isoBrushSize);
					brush.drawAtAbsolutePos(v, isoBrushDensity);
				}
			}
		}
	}

	public void keyPressed() {
		if (key=='s'){
			meshA.saveAsSTL(sketchPath(meshA.name + frameCount+ counter + ".stl"));
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
		if (key == 'c') {
			compute = !compute;
		}
	}


	public static void main(String _args[]) {
		PApplet.main(new String[] { multiagentgeometryexploringsystem.MultiAgentGeometryExploringSystem.class.getName() });
	}
}
