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
 * @author James MacGlashan.
 */
public class SkeletonGW {

	public static void main(String[] args) {

		GridWorldDomain gwd = new GridWorldDomain(10, 10); //we'll give ourselves a 10x10 gridWorld canvas
		gwd.setNumberOfLocationTypes(2); //two kinds of locations to specify
		gwd.horizontal1DNorthWall(0, 9, 1); //make a wall to confine space to the bottom two rows
		gwd.setProbSucceedTransitionDynamics(1.00); // make things noisy
// -1000: .5 ?, .6 ?, .62 E, .64 S, .66 W,        .70 ?,                      .82 W
// -100:  .5 E, .6 E, .62 E, .64 S, .66 W,        .70 W
// -10:   .5 E, .6 E, .62 E, .64 S, .66 W,        .70 W
// -1:    .5 ?, .6 E, .62 E, .64 E, .66 S, .68 W, .70 W
// -.9:   .5 ?, .6 ?, .62 ?, .64 E, .66 S, .68 W, .70 ?
// -.8:   .5 ?, .6 ?, .62 ?, .64 E, .66 S, .68 W, .70 ?
// -.7:   .5 ?, .6 ?, .62 ?, .64 ?, .66 E, .68 W, .70 ?
// -.6:   .5 ?, .6 ?, .62 ?, .64 ?, .66 E, .68 W, .70 ?
// -.5:   .5 ?, .6 ?, .62 ?, .64 ?, .66 E, .68 W, .70 ?
// -.4:   .5 ?, .6 ?, .62 ?, .64 ?, .66 ?, .68 E, .70 W
// -.3:   .5 ?, .6 ?, .62 ?, .64 ?, .66 ?, .68 E, .70 W
// -.2:   .5 ?, .6 ?,                             .70 E, .72 E, .74 W, .80 W, .82 ?, .90 ?
// -.1:   .5 ?, .6 E,                             .70 E,               .80 E, .82 W, .90 W
// -.01:  .5 ?, .6 ?,                             .70 ?,               .80 ?, .82 ?, .90 E, .94 E, .96 E, .98 E, 1.00 N
		// gwd.setTransitionDynamics(new double[][]{{0.8, 0., 0.1, 0.1}, {0., 0.8, 0.1, 0.1}, {0.1, 0.1, 0.8, 0.}, {0.1, 0.1, 0., 0.8}});
		final Domain domain = gwd.generateDomain(); //create our domain

		//construct our state
		int numLocations = 10; //I'm only adding two locations for illustrative purposes
		State s = GridWorldDomain.getOneAgentNLocationState(domain,numLocations);
		GridWorldDomain.setAgent(s, 7, 0); //agent starts at 7,0
		GridWorldDomain.setLocation(s, 0, 0, 1, 1); //first location (0) in 0,1 with type 1

		int i;
		for (i = 0; i < 6; i++) {
		    GridWorldDomain.setLocation(s, i+1, i+1, 1, 0); //second location (1) in 1,1 with type 0
		    // System.out.println("Count is: " + i);
		}
		GridWorldDomain.setLocation(s, 7, 8, 0, 0);
		GridWorldDomain.setLocation(s, 8, 8, 1, 0);
		GridWorldDomain.setLocation(s, 9, 9, 1, 1);
		
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
							return 1.;
						}
						else if(locationType == 0){
							//puddle location
							return -.01;
						}

						break; //break; agent cannot be at multiple locations at once
					}
				}


				return 0.; //default reward for not being in any location
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