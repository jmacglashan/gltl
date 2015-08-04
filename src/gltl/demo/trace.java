package gltl.demo;

import Tools.EpisodeRenderer;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.RewardFunction;
import gltl.compiler.GLTLCompiler;
import burlap.oomdp.visualizer.Visualizer;
import burlap.oomdp.visualizer.StaticPainter;



import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.awt.Graphics2D;

/**
 * @author James MacGlashan. ... tweaker Michael Littman
 */
public class trace {

	public static final String PFINORANGE = "InOrange";
	public static final String PFINBLUE = "InBlue";
	public static final String PFNOTINBLUE = "NotInBlue";

	public static void main(String[] args) {

		String formula;
		char grid;


		formula = "U6!BR"; // avoid blue en route to red
		grid = 'R';
		// formula = "U6!BB"; // avoid blue en route to blue
//		formula = "&G2!BF2R"; // eventually red and always not blue (waits)
//		formula = "F3B"; // go to blue
		formula = "F2R"; // go to red
		formula = "G3!B"; // avoid blue

		grid = 'H';
		formula = "&F3RG1F1B"; // hallway!

		grid = 'N';
		formula = "U2!BR"; // avoid blue en route to red
		formula = "G2F2&BF2R"; // slides
		formula = "&F2RG2!B";  // slides

//		formula = "!F3!F3B"; // always eventually blue
//		formula = "F3&RF3B"; // go to red then blue
//		formula = "F3&BF3R"; // go to blue then red
//		formula = "F3&RF3&BF3R"; // red, blue, red
//		formula = "&F3RF3B";    // has to touch both
//		formula = "G1F1R";		// always eventually red
//		formula = "F1G1R";		// eventually always red (ever so slightly different)
//		formula = "G4F4B"; // stay in blue (always eventually) (also slow)
//		formula = "G3F3&RF3B"; // red, blue, repeat (planning takes awhile)
//		formula = "F1F1R";	// eventually-eventually...
//		formula = "F2R";	// eventually... same as above!
//		formula = "G2&F2RF2B"; // always go to red and always go to blue (does red-red-blue-blue-red-red-blue-blue...)
//		formula = "&G1F2RG1F2B"; // does a bunch of blue then a bunch of red. not sure why it doesn't intermingle
//		formula = "|G1F1RG4F4B"; // Formula from Charles' talk
//		formula = "&G1F1RG4F4B"; // try "and"
//		formula = "&F1RG1F1B"; // hallway with short time to graduate.
//		formula = "&F1RG1!B"; // avoid blue
//		formula = "U3!BR"; // not blue until red

		// formula = eventually(make(P))
		// GLTLCompiler.TransitionQuery formulaobj = new GLTLCompiler.TransitionQuery()

		if(args.length > 0){
			formula = args[0];
		}

		System.out.println("Running for formula " + formula);

		GridWorldDomain gwd;
		final Domain envDomain;
		int numLocations;
		State s;

		switch (grid) {
			case 'R': // Rescue.
				//define our environment MDP
				// GridWorldDomain gwd = new GridWorldDomain(11, 11);
				gwd = new GridWorldDomain(10, 10);
				//gwd.setMapToFourRooms();
				gwd.setNumberOfLocationTypes(2); //two kinds of locations to specify
				gwd.setProbSucceedTransitionDynamics(0.80);

				envDomain = gwd.generateDomain();

				//construct our state
				numLocations = 24; // allocate locations
				s = GridWorldDomain.getOneAgentNLocationState(envDomain, numLocations);
				GridWorldDomain.setAgent(s, 1, 7); //agent starting place
				// GridWorldDomain.setAgent(s, 1, 1); //agent starting place initial location is blue
				GridWorldDomain.setLocation(s, 0, 7, 8, 1); //first location (0) in 0,1 with type 1 = goal

				int i;
				for (i = 0; i < 3; i++) {
					GridWorldDomain.setLocation(s, i + 1, i + 1, 1, 0); //second location (1) in 1,1 with type 0
					// System.out.println("Count is: " + i);
				}
				for (i = 0; i < 4; i++) {
					GridWorldDomain.setLocation(s, i + 4, i + 4, 2, 0);
				}
				for (i = 0; i < 7; i++) {
					GridWorldDomain.setLocation(s, i + 8, 3, i + 3, 0);
				}
				for (i = 0; i < 4; i++) {
					GridWorldDomain.setLocation(s, i + 15, 5, i + 5, 0);
				}
				for (i = 0; i < 3; i++) {
					GridWorldDomain.setLocation(s, i + 19, 7 + i, 5 + i, 0);
				}
				GridWorldDomain.setLocation(s, 22, 6, 5, 0);
				GridWorldDomain.setLocation(s, 23, 9, 8, 0);

				// State initialEnvState = GridWorldDomain.getOneAgentNoLocationState(envDomain, 0, 0);
				break;
			case 'H': // hallway
				//define our environment MDP
				// GridWorldDomain gwd = new GridWorldDomain(11, 11);
				gwd = new GridWorldDomain(5, 5);
				//gwd.setMapToFourRooms();
				gwd.setNumberOfLocationTypes(2); //two kinds of locations to specify
				//gwd.setProbSucceedTransitionDynamics(0.80);
				gwd.horizontalWall(0, 4, 2); // upper wall
				gwd.horizontalWall(1,1,0); // internal wall
				gwd.horizontalWall(3,3,0); // internal wall

				envDomain = gwd.generateDomain();

				//construct our state
				numLocations = 3; // allocate locations
				s = GridWorldDomain.getOneAgentNLocationState(envDomain, numLocations);
				GridWorldDomain.setAgent(s, 0, 1); //agent starting place
				GridWorldDomain.setLocation(s, 0, 0, 0, 0); //first location (0) with type 1 = blue
				GridWorldDomain.setLocation(s, 1, 2, 0, 0); //second location with type 1 = blue

				GridWorldDomain.setLocation(s, 2, 4, 1, 1); // goal

				break;

			case 'N': // Russell Norvig grid 3x4
			default:
				//define our environment MDP
				// GridWorldDomain gwd = new GridWorldDomain(11, 11);
				gwd = new GridWorldDomain(4, 4);
				//gwd.setMapToFourRooms();
				gwd.setNumberOfLocationTypes(2); //two kinds of locations to specify
				gwd.setProbSucceedTransitionDynamics(0.80);
				gwd.horizontalWall(0,3,3); // upper wall
				gwd.horizontalWall(1,1,1); // internal wall

				envDomain = gwd.generateDomain();

				//construct our state
				numLocations = 2; // allocate locations
				s = GridWorldDomain.getOneAgentNLocationState(envDomain, numLocations);
				GridWorldDomain.setAgent(s, 0, 0); //agent starting place
				GridWorldDomain.setLocation(s, 0, 3, 1, 0); //first location (0) with type 1 = goal
				GridWorldDomain.setLocation(s, 1, 3, 2, 1); //second location (1) with type 1 = blue

				break;
		}

		//add propositional function for checking if the agent is in an orange or blue cell
		new InOrange(envDomain);
		new InBlue(envDomain);
		new NotInBlue(envDomain);

		//let our GLTL symbol "o" correspond to evaluating whether the agent is in an orange location (a parameterless propositional function)
		// and "b" be whether the agent is in a blue location
		Map<String, PropositionalFunction> symbolMap = new HashMap<String, PropositionalFunction>(1);
		//System.out.println(envDomain.getPropFunctions());
		symbolMap.put("R", envDomain.getPropFunction(PFINORANGE));
		symbolMap.put("B", envDomain.getPropFunction(PFINBLUE));
		// symbolMap.put("R", envDomain.getPropFunction(PFNOTINBLUE));
		//System.out.println(symbolMap.toString());


		//construct our GLTL compiler for our grid world environment and the given formula (currently hardcoded by numbers)
		GLTLCompiler compiler = new GLTLCompiler(formula, symbolMap, envDomain);
		Domain compiledDomain = compiler.generateDomain();
		RewardFunction rf = compiler.generateRewardFunction();
		TerminalFunction tf = compiler.generateTerminalFunction();

		State initialCompiledState = compiler.addInitialTaskStateToEnvironmentState(compiledDomain, s);
		// System.out.println(initialCompiledState.getCompleteStateDescription());

		//begin planning in our compiled domain
		ValueIteration vi = new ValueIteration(compiledDomain, rf, tf, 1.0, new DiscreteStateHashFactory(), 0.0000001, 20000);
		vi.toggleReachabiltiyTerminalStatePruning(true);
//		System.out.println(initialCompiledState.toString());
//		System.out.println(vi.getQ(initialCompiledState, compiledDomain.getSingleAction("N")));
		vi.planFromState(initialCompiledState);

		Policy p = new GreedyQPolicy(vi);
		EpisodeAnalysis ea = p.evaluateBehavior(initialCompiledState, rf, tf);

		Visualizer v = GridWorldVisualizer.getVisualizer(gwd.getMap());
		StaticPainter sp = new StaticPainter() {
			@Override
			public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {
				ObjectInstance taskObject = s.getFirstObjectOfClass(GLTLCompiler.CLASSSPEC);
				if(taskObject != null) {
					System.out.println(taskObject.getObjectDescription());
				}
			}
		};
		v.addStaticPainter(sp);

//		new EpisodeSequenceVisualizer(v, compiledDomain, episodes);
//
//		//create episode sequence visualizer
//		Visualizer v = GridWorldVisualizer.getVisualizer(gwd.getMap());
		EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, compiledDomain, Arrays.asList(ea));
////		EpisodeRenderer evis = new EpisodeRenderer(v, compiledDomain, Arrays.asList(ea));

	}



	public static class InOrange extends PropositionalFunction {

		public InOrange(Domain domain) {
			super(PFINORANGE, domain, "");
		}

		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance agent = s.getFirstObjectOfClass(GridWorldDomain.CLASSAGENT);
			int x = agent.getIntValForAttribute(GridWorldDomain.ATTX);
			int y = agent.getIntValForAttribute(GridWorldDomain.ATTY);

			List<ObjectInstance> locations = s.getObjectsOfClass(GridWorldDomain.CLASSLOCATION);
			for (ObjectInstance location : locations) {
				int lx = location.getIntValForAttribute(GridWorldDomain.ATTX);
				int ly = location.getIntValForAttribute(GridWorldDomain.ATTY);

				if ((lx == x) && (ly == y) && (location.getIntValForAttribute(GridWorldDomain.ATTLOCTYPE) == 1)) {
					return true;
				}
			}
			return false;
		}

	}

	public static class InBlue extends PropositionalFunction {

		public InBlue(Domain domain) {
			super(PFINBLUE, domain, "");
		}

		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance agent = s.getFirstObjectOfClass(GridWorldDomain.CLASSAGENT);
			int x = agent.getIntValForAttribute(GridWorldDomain.ATTX);
			int y = agent.getIntValForAttribute(GridWorldDomain.ATTY);

			List<ObjectInstance> locations = s.getObjectsOfClass(GridWorldDomain.CLASSLOCATION);
			for (ObjectInstance location : locations) {
				int lx = location.getIntValForAttribute(GridWorldDomain.ATTX);
				int ly = location.getIntValForAttribute(GridWorldDomain.ATTY);

				if ((lx == x) && (ly == y) && (location.getIntValForAttribute(GridWorldDomain.ATTLOCTYPE) == 0)) {
					return true;
				}
			}
			return false;
		}

	}

	public static class NotInBlue extends PropositionalFunction {

		public NotInBlue(Domain domain) {
			super(PFNOTINBLUE, domain, "");
		}

		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance agent = s.getFirstObjectOfClass(GridWorldDomain.CLASSAGENT);
			int x = agent.getIntValForAttribute(GridWorldDomain.ATTX);
			int y = agent.getIntValForAttribute(GridWorldDomain.ATTY);

			List<ObjectInstance> locations = s.getObjectsOfClass(GridWorldDomain.CLASSLOCATION);
			for (ObjectInstance location : locations) {
				int lx = location.getIntValForAttribute(GridWorldDomain.ATTX);
				int ly = location.getIntValForAttribute(GridWorldDomain.ATTY);

				if ((lx == x) && (ly == y) && (location.getIntValForAttribute(GridWorldDomain.ATTLOCTYPE) == 0)) {
					return false;
				}
			}
			return true;
		}

	}
}
