package gltl.demo;

import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.visualizer.Visualizer;

import java.util.Arrays;
import java.util.List;

/**
 * @author James MacGlashan. (with broken stuff introduced by Michael.)
 */
public class ten {

	public static void main(String[] args) {

		GridWorldDomain gwd = new GridWorldDomain(10, 10); //we'll give ourselves a 10x10 gridWorld canvas
		gwd.setNumberOfLocationTypes(2); //two kinds of locations to specify
		// gwd.horizontal1DNorthWall(0, 9, 1); //make a wall to confine space to the bottom two rows
		gwd.setProbSucceedTransitionDynamics(0.95); // make things noisy slip
		final Domain domain = gwd.generateDomain(); //create our domain

		//construct our state
		int numLocations = 24; // allocate locations
		State s = GridWorldDomain.getOneAgentNLocationState(domain,numLocations);
		GridWorldDomain.setAgent(s, 1, 7); //agent starting place
		GridWorldDomain.setLocation(s, 0, 7, 8, 1); //first location (0) in 0,1 with type 1 = goal

		int i;
		for (i = 0; i < 3; i++) {
		    GridWorldDomain.setLocation(s, i+1, i+1, 1, 0); //second location (1) in 1,1 with type 0
		    // System.out.println("Count is: " + i);
		}
		for (i = 0; i < 4; i++) {
		    GridWorldDomain.setLocation(s, i+4, i+4, 2, 0); 
		}
		for (i = 0; i < 7; i++) {
		    GridWorldDomain.setLocation(s, i+8, 3, i+3, 0); 
		}
		for (i = 0; i < 4; i++) {
		    GridWorldDomain.setLocation(s, i+15, 5, i+5, 0);
		}
		for (i = 0; i < 3; i++) {
		    GridWorldDomain.setLocation(s, i+19, 7+i, 5+i, 0);
		}
		GridWorldDomain.setLocation(s, 22, 6, 5, 0);
		GridWorldDomain.setLocation(s, 23, 9, 8, 0);
		
		//uncomment to launch visual explorer
		//stateExplorer(gwd, domain, s);
		
		//define a reward function for going to location type 1 and avoiding type 0
		RewardFunction rf = new RewardFunction() {
			@Override
			public double reward(State s, GroundedAction a, State sprime) {

				//get the "at location" propositional function
				PropositionalFunction pf = domain.getPropFunction(GridWorldDomain.PFATLOCATION);
				//get all possible location bindings in the next state
				List<GroundedProp> gps = pf.getAllGroundedPropsForState(sprime);
				//check if the agent is in any of those locations
				for(GroundedProp gp : gps){
					if(gp.isTrue(sprime)){
						//the agent is at this location; what type of location is it?
						//the name of the object is in the second parameter of the proposition function ("atLocation(agent, location)")
						ObjectInstance location = sprime.getObject(gp.params[1]);
						int locationType = location.getIntValForAttribute(GridWorldDomain.ATTLOCTYPE);
						if(locationType == 1){
							//goal location
							return 0.;
						}
						else if(locationType == 0){
							//puddle location
							return -70.;
						}

						break; //break; agent cannot be at multiple locations at once
					}
				}


				return -1.; //default reward for not being in any location
			}
		};

		//define a terminal function for reaching location type 1
		TerminalFunction tf = new TerminalFunction() {
			@Override
			public boolean isTerminal(State s) {
				//get the "at location" propositional function
				PropositionalFunction pf = domain.getPropFunction(GridWorldDomain.PFATLOCATION);
				//get all possible location bindings in the next state
				List<GroundedProp> gps = pf.getAllGroundedPropsForState(s);
				//check if the agent is in any of those locations
				for(GroundedProp gp : gps) {
					if(gp.isTrue(s)) {
						//the agent is at this location; what type of location is it?
						//the name of the object is in the second parameter of the proposition function ("atLocation(agent, location)")
						ObjectInstance location = s.getObject(gp.params[1]);
						int locationType = location.getIntValForAttribute(GridWorldDomain.ATTLOCTYPE);
						if(locationType == 1) {
							return true;
						}
						break;
					}

				}

				return false;
			}

		};

		//run planning to visualize
		planAndVisualize(gwd, domain, rf, tf, 0.99, s);
	}

	public static void stateExplorer(GridWorldDomain gwd, Domain domain, State s){
		Visualizer v = GridWorldVisualizer.getVisualizer(gwd.getMap());
		VisualExplorer exp = new VisualExplorer(domain, v, s);
		exp.addKeyAction("w", GridWorldDomain.ACTIONNORTH);
		exp.addKeyAction("s", GridWorldDomain.ACTIONSOUTH);
		exp.addKeyAction("a", GridWorldDomain.ACTIONWEST);
		exp.addKeyAction("d", GridWorldDomain.ACTIONEAST);
		exp.initGUI();;
	}


	public static void planAndVisualize(GridWorldDomain gwd, Domain domain, RewardFunction rf, TerminalFunction tf, double discountFactor, State initialState){

		//use value iteration
		ValueIteration vi = new ValueIteration(domain, rf, tf, discountFactor, new DiscreteStateHashFactory(), 0.01, 100);
		vi.planFromState(initialState);
		Policy p = new GreedyQPolicy(vi);
		EpisodeAnalysis ea = p.evaluateBehavior(initialState, rf, tf);

		//crate episode sequence visualizer
		Visualizer v = GridWorldVisualizer.getVisualizer(gwd.getMap());
		EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, domain, Arrays.asList(ea));

	}
}
