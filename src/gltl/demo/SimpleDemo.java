package gltl.demo;

import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import gltl.compiler.GLTLCompiler;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author James MacGlashan. ... tweaker Michael Littman
 */
public class SimpleDemo {

	public static final String PFINSOUTHWEST = "InSouthWestRoom";

	public static void main(String[] args) {

		String formula = "4";
		if(args.length > 0){
			formula = args[0];
		}

		System.out.println("Running for formula " + formula);

		//define our environment MDP
		GridWorldDomain gwd = new GridWorldDomain(11, 11);
		gwd.setMapToFourRooms();
		gwd.setProbSucceedTransitionDynamics(0.8);

		Domain envDomain = gwd.generateDomain();

		//add propositional function for checking if the agent is in the south west room
		new SouthWestRoomPF(envDomain);

		//get initial environment state
		State initialEnvState = GridWorldDomain.getOneAgentNoLocationState(envDomain, 10, 10);
		if(formula.equals("2")){
			initialEnvState = GridWorldDomain.getOneAgentNoLocationState(envDomain, 0, 0);
		}

		//let our GLTL symbol "p" correspond to evaluating whether the agent is in the south west room (a parameterless propositional function)
		Map<String, PropositionalFunction> symbolMap = new HashMap<String, PropositionalFunction>(1);
		symbolMap.put("P", envDomain.getPropFunction(PFINSOUTHWEST));

		//construct our GLTL compiler for our grid world environment and the given formula (currently hardcoded by numbers)
		GLTLCompiler compiler = new GLTLCompiler(formula, symbolMap, envDomain);
		Domain compiledDomain = compiler.generateDomain();
		RewardFunction rf = compiler.generateRewardFunction();
		TerminalFunction tf = compiler.generateTerminalFunction();

		State initialCompiledState = compiler.addInitialTaskStateToEnvironmentState(compiledDomain, initialEnvState);


		//begin planning in our compiled domain
		ValueIteration vi = new ValueIteration(compiledDomain, rf, tf, 0.99, new DiscreteStateHashFactory(), 0.001, 200);
		vi.planFromState(initialCompiledState);


		//for value function visualization, lets visualize all reachable states from our initial compiled state,
		//but filter out any that are not initial states of the task
		List <State> allCompiledStates = StateReachability.getReachableStates(initialCompiledState, (SADomain)compiledDomain, new DiscreteStateHashFactory());
		List <State> compiledStates = new ArrayList<State>(allCompiledStates.size());
		for(State s : allCompiledStates){
			int specVal = s.getFirstObjectOfClass(GLTLCompiler.CLASSSPEC).getIntValForAttribute(GLTLCompiler.ATTSPEC);
			if(specVal == 0) {
				compiledStates.add(compiler.addInitialTaskStateToEnvironmentState(compiledDomain, s));
			}
		}

		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(compiledStates, vi, new GreedyQPolicy(vi));
		gui.initGUI();


	}


	public static class SouthWestRoomPF extends PropositionalFunction{

		// This is for safety
		protected int [][] safetyMap = new int[][]{
				{2,2,2,2,2,1,0,0,0,0,0},
				{2,2,2,2,2,0,0,0,0,0,0},
				{2,2,2,2,2,1,0,0,0,0,0},
				{2,2,2,2,2,1,0,0,0,0,0},
				{2,2,2,2,2,1,0,0,0,0,0},
				{1,0,1,1,1,1,1,1,0,1,1},
				{0,0,0,0,1,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,0,0,0,0},
		};

		public SouthWestRoomPF(Domain domain) {
			super(PFINSOUTHWEST, domain, "");
		}

		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance agent = s.getFirstObjectOfClass(GridWorldDomain.CLASSAGENT);
			int x = agent.getIntValForAttribute(GridWorldDomain.ATTX);
			int y = agent.getIntValForAttribute(GridWorldDomain.ATTY);


			return this.safetyMap[x][y] == 2;
		}
	}

}
