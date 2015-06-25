package gltl.prototype;

import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldStateParser;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.VisualActionObserver;
import burlap.oomdp.visualizer.ObjectPainter;
import burlap.oomdp.visualizer.StateRenderLayer;
import burlap.oomdp.visualizer.StaticPainter;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.visualizer.Visualizer;

import java.awt.*;
import java.util.List;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * @author James MacGlashan.
 */
public class EnvironmentMDP implements DomainGenerator{

	public static final String ATTX = "x";
	public static final String ATTY = "y";
	// wm
	public static final String ATTSPEC = "spec";
	public static final String ATTSPECTRUE = "specTrue";
	public static final String ATTSPECFALSE = "specFalse";
	public static final String ATTFORMULA = "specformula";
	public static final String ATTPARAM = "specParameter";

	public static final String CLASSAGENT = "agent";
	public static final String CLASSLOCATION = "location";
	// wm
	public static final String CLASSSPEC = "spec";

	public static final String ACTIONNORTH = "north";
	public static final String ACTIONSOUTH = "south";
	public static final String ACTIONEAST = "east";
	public static final String ACTIONWEST = "west";

	public static final String PFAT = "at";

	// wm
	public String gltlSpec = "";
	int numTotalFormula = 0;
	public SADomain domain = new SADomain();
	public DiscreteStateHashFactory hashingFactory = new DiscreteStateHashFactory();
	public StateParser sp;
	public AbsorbingState tf;
	public Reward rf;

	//ordered so first dimension is x
	protected int [][] map = new int[][]{
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,0,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{1,0,1,1,1,1,1,1,0,1,1},
			{0,0,0,0,1,0,0,0,0,0,0},
			{0,0,0,0,1,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0,0,0,0},
			{0,0,0,0,1,0,0,0,0,0,0},
			{0,0,0,0,1,0,0,0,0,0,0},
	};

	// 2 for safe states (Up to now, it labels the states where p holds).
	// Note that this map is not exactly the same as the visualization:
	// The visualization can be acquired by rotating this matrix by 90 degrees counterclockwise.

	/*
	// This is for reachability
	protected int [][] safetyMap = new int[][]{
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,0,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{1,0,1,1,1,1,1,1,0,1,1},
			{0,0,0,0,1,0,0,0,0,0,0},
			{0,0,0,0,1,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0,0,0,0},
			{0,0,0,0,1,0,0,0,0,0,0},
			{0,0,0,0,1,0,0,0,0,0,2},
	};
	 */


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

	/*
	// This is for safety
	protected int [][] safetyMap = new int[][]{
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,0,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{0,0,0,0,0,1,0,0,0,0,0},
			{1,0,1,1,1,1,1,1,0,1,1},
			{2,2,2,2,1,0,0,0,0,0,0},
			{2,2,2,2,1,0,0,0,0,0,0},
			{2,2,2,2,0,0,0,0,0,0,0},
			{2,2,2,2,1,0,0,0,0,0,0},
			{2,2,2,2,1,0,0,0,0,0,0},
	};
	 */

	@Override
	public Domain generateDomain() {

		// wm
		// SADomain domain = new SADomain();

		Attribute xAtt = new Attribute(this.domain, ATTX, AttributeType.INT);
		xAtt.setLims(0, 10);

		Attribute yAtt = new Attribute(this.domain, ATTY, AttributeType.INT);
		yAtt.setLims(0, 10);

		ObjectClass agentClass = new ObjectClass(this.domain, CLASSAGENT);
		agentClass.addAttribute(xAtt);
		agentClass.addAttribute(yAtt);

		new Movement(ACTIONNORTH, this.domain, 0);
		new Movement(ACTIONSOUTH, this.domain, 1);
		new Movement(ACTIONEAST, this.domain, 2);
		new Movement(ACTIONWEST, this.domain, 3);

		new AtLocation(this.domain);

		return this.domain;

	}

	protected class Movement extends Action {

		//0: north; 1: south; 2:east; 3: west
		protected double [] directionProbs = new double[4];
		// wm

		// Transition distribution to all successors when taking each action.
		public Movement(String actionName, Domain domain, int direction) {
			// A constructor of the superclass Action
			super(actionName, domain, "");
			//** Change transition distribution here.
			for(int i = 0; i < 4; i++){
				if(i == direction){
					directionProbs[i] = 0.8;
				}
				else{
					directionProbs[i] = 0.2/3.;
				}
			}

		}

		// wm
		// consider the transition probability for the spec MDP
		// return the probability that the corresponding successor is taken.
		public double SpecMoveDistribution(State s, State sNew){
			// newPos could be used in state transition of the specification MDP

			ObjectInstance agentSpec = s.getFirstObjectOfClass(CLASSSPEC);

			int curStateSpec = agentSpec.getIntValForAttribute(ATTSPEC);
			int accSpec = agentSpec.getIntValForAttribute(ATTSPECTRUE);
			int rejSpec = agentSpec.getIntValForAttribute(ATTSPECFALSE);
			int formulaSpec = agentSpec.getIntValForAttribute(ATTFORMULA);
			int [] paramSpec = agentSpec.getIntArrayValForAttribute(ATTPARAM);

			ObjectInstance agentNewSpec = sNew.getFirstObjectOfClass(CLASSSPEC);
			int newStateSpec = agentNewSpec.getIntValForAttribute(ATTSPEC);

			ObjectInstance agentNewPos = sNew.getFirstObjectOfClass(CLASSAGENT);
			int xNew = agentNewPos.getIntValForAttribute(ATTX);
			int yNew = agentNewPos.getIntValForAttribute(ATTY);

			// the AP p is satisfied: goalX == newPos[0] && goalY == newPos[1]
			if (curStateSpec == accSpec || curStateSpec == rejSpec){
				// absorbing states
				return 1.0;
			}
			else{
				// Neither accepted nor rejected yet
				if (formulaSpec == 1){
					// F k p
					if ((newStateSpec == accSpec)  && (isP(xNew, yNew))){
						return 1.0;
					}
					else if (newStateSpec == accSpec && (! isP(xNew, yNew))){
						return 0.0;
					}
					else if (newStateSpec == rejSpec && (! isP(xNew, yNew))){
						return 1.0 / (double) paramSpec[0];
					}
					else if (newStateSpec == rejSpec && (isP(xNew, yNew))){
						return 0.0;
					}
					else if (isP(xNew, yNew)){
						return 0.0;
					}
					else{
						return (double)(paramSpec[0] - 1.0) / (double)paramSpec[0];
					}
				}
				else if (formulaSpec == 2){
					// G k p
					if (newStateSpec == rejSpec && (! isP(xNew, yNew))){
						return 1.0;
					}
					else if (newStateSpec == rejSpec && isP(xNew, yNew)){
						return 0.0;
					}
					else if (newStateSpec == accSpec && isP(xNew, yNew)){
						return 1.0 / (double) paramSpec[0];
					}
					else if (newStateSpec == rejSpec && (! isP(xNew, yNew))){
						return 0.0;
					}
					else if (! isP(xNew, yNew)){
						return 0.0;
					}
					else{
						return (double)(paramSpec[0] - 1.0) / (double)paramSpec[0];
					}
				}
				else if (formulaSpec == 3){
					// F l (G k p)
					// k == paramSpec[0]
					// l == paramSpec[1]
					double k = (double) paramSpec[0];
					double l = (double) paramSpec[1];

					ObjectInstance agentNew = sNew.getFirstObjectOfClass(CLASSAGENT);
					int newPosX = agentNew.getIntValForAttribute(ATTX);
					int newPosY = agentNew.getIntValForAttribute(ATTY);

					if (curStateSpec == 0){
						// still at the initial state

						if (isP(newPosX, newPosY)){
							// p
							double [] specTransitionProb = {(k-1)*(l-1)/k/l, (k-1)/k/l, 1/k};
							int [] specTransitonSuccessor = {0, 3, accSpec};
							for (int i = 0; i < specTransitionProb.length; i++){
								if (newStateSpec == specTransitonSuccessor[i]){
									return specTransitionProb[i];
								}
							}
							System.out.println("Unexpected spec successor: numFormula = 3");
						}
						else{
							// not p
							if (newStateSpec == rejSpec){
								return 1.0 / l;
							}
							else if (newStateSpec == curStateSpec){
								return 1.0 - 1.0 / l;
							}
							else{
								System.out.println("Unexpected spec successor, numFormula = 3, curStateSpec == 0");
								return 0.0;
							}
						}
					}
					else if (curStateSpec == 3){
						// the second non-terminating state, which is the last chance to satisfy the spec
						if (newStateSpec == accSpec && (isP(newPosX, newPosY))){
							return 1.0 / k;
						}
						else if (newStateSpec == curStateSpec && isP(newPosX, newPosY)){
							return 1.0 - 1.0 / k;
						}
						else if (newStateSpec == rejSpec && ! (isP(newPosX, newPosY))){
							return 1.0;
						}
						else{
							System.out.println("Unexpected spec successor, numFormula = 3, curStateSpec == 3");
							return 0.0;
						}
					}
				}
				else if (formulaSpec == 4){
					// G l (F k p)
					// k == paramSpec[0]
					// l == paramSpec[1]
					double k = (double) paramSpec[0];
					double l = (double) paramSpec[1];

					ObjectInstance agentNew = sNew.getFirstObjectOfClass(CLASSAGENT);
					int newPosX = agentNew.getIntValForAttribute(ATTX);
					int newPosY = agentNew.getIntValForAttribute(ATTY);
					if (curStateSpec == 0){
						// still at the initial state

						if (! isP(newPosX, newPosY)){
							// not p
							double [] specTransitionProb = {(k-1)*(l-1)/k/l, (l-1)/k/l, 1/l};
							int [] specTransitonSuccessor = {0, 3, rejSpec};
							for (int i = 0; i < specTransitionProb.length; i++){
								if (newStateSpec == specTransitonSuccessor[i]){
									return specTransitionProb[i];
								}
							}
							System.out.println("Unexpected spec successor: numFormula = 4");
						}
						else{
							// p
							if (newStateSpec == accSpec && isP(newPosX, newPosY)){
								return 1.0/k;
							}
							else if (newStateSpec == curStateSpec && isP(newPosX, newPosY)){
								return 1.0 - 1.0 / k;
							}
							else {
								System.out.println("Unexpected spec successor, numFormula = 4, curStateSpec == 0");
								return 0.0;
							}
						}
					}
					else if (curStateSpec == 3){
						// the second non-terminating state, which is the last chance to satisfy the spec
						if (newStateSpec == accSpec && isP(newPosX, newPosY)){
							return 1.0;
						}
						else if (newStateSpec == rejSpec && !(isP(newPosX, newPosY))){
							return 1.0 / l;
						}
						else if (!(isP(newPosX, newPosY))){
							return 1.0 - 1.0 / l;
						}
						else{
							System.out.println("Unexpected spec successor, numFormula = 4, curStateSpec == 3");
							return 0.0;
						}
					}
				}
				else{
					System.out.println("[SpecMoveDistribution] Error: invalid formula.");
				}
			}
			return -1;
		}

		// wm
		// the transition function of the specification MDP
		// returns a list of possible successors when the successor of the state s is at newPos.
		public State [] SpecMoveResult(State s, int[] newPos){
			State ss = s.copy();
			// newPos could be used in state transition of the specification MDP
			ObjectInstance agentSpec = ss.getFirstObjectOfClass(CLASSSPEC);

			int curStateSpec = agentSpec.getIntValForAttribute(ATTSPEC);
			int accSpec = agentSpec.getIntValForAttribute(ATTSPECTRUE);
			int rejSpec = agentSpec.getIntValForAttribute(ATTSPECFALSE);
			int formulaSpec = agentSpec.getIntValForAttribute(ATTFORMULA);
			//int [] paramSpec = agentSpec.getIntArrayValForAttribute(ATTPARAM);

			if (curStateSpec == accSpec || curStateSpec == rejSpec){
				State [] sList = {ss};
				return sList;
			}
			else{
				// get the goal position, which is initialized in getNewState
				// Neither accepted nor rejected yet
				if (formulaSpec == 1){
					// F k p
					if (isP(newPos[0], newPos[1])){
						// p
						agentSpec.setValue(ATTSPEC, accSpec);
						State [] sList = {ss};
						return sList;
					}
					else{
						// not p
						State ss2 = ss.copy();
						agentSpec.setValue(ATTSPEC, rejSpec);
						State [] sList = {ss, ss2};
						return sList;
					}
				}
				else if (formulaSpec == 2){
					// G k p
					if (! isP(newPos[0], newPos[1])){
						// not p
						agentSpec.setValue(ATTSPEC, rejSpec);
						State [] sList = {ss};
						return sList;
					}
					else{
						// p
						State ss2 = ss.copy();
						agentSpec.setValue(ATTSPEC, accSpec);
						State [] sList = {ss, ss2};
						return sList;
					}
				}
				else if (formulaSpec == 3){
					// F l (G k p)
					if (curStateSpec == 0){
						// still at the initial state
						if (isP(newPos[0], newPos[1])){
							// p
							State ss2 = ss.copy();
							agentSpec.setValue(ATTSPEC, 3);
							State ss3 = ss.copy();
							agentSpec.setValue(ATTSPEC, accSpec);
							State[] sList = {ss, ss2, ss3};
							return sList;
						}
						else{
							// not p
							State ss2 = ss.copy();
							agentSpec.setValue(ATTSPEC, rejSpec);
							State [] sList = {ss, ss2};
							return sList;
						}
					}
					else if (curStateSpec == 3){
						// the second non-terminating state, which is the last chance to satisfy the spec
						if (isP(newPos[0], newPos[1])){
							// p
							State ss2 = ss.copy();
							agentSpec.setValue(ATTSPEC, accSpec);
							State [] sList = {ss, ss2};
							return sList;
						}
						else{
							// not p
							agentSpec.setValue(ATTSPEC, rejSpec);
							State [] sList = {ss};
							return sList;
						}
					}
				}
				else if (formulaSpec == 4){
					// G l (F k p)
					if (curStateSpec == 0){
						// still at the initial state
						if (! isP(newPos[0], newPos[1])){
							// not p
							State ss2 = ss.copy();
							agentSpec.setValue(ATTSPEC, 3);
							State ss3 = ss.copy();
							agentSpec.setValue(ATTSPEC, rejSpec);
							State[] sList = {ss, ss2, ss3};
							return sList;
						}
						else{
							// p
							State ss2 = ss.copy();
							agentSpec.setValue(ATTSPEC, accSpec);
							State [] sList = {ss, ss2};
							return sList;
						}

					}
					else if (curStateSpec == 3){
						// the second non-terminating state, which is the last chance to satisfy the spec
						if (! isP(newPos[0], newPos[1])){
							// not p
							State ss2 = ss.copy();
							agentSpec.setValue(ATTSPEC, rejSpec);
							State [] sList = {ss, ss2};
							return sList;
						}
						else{
							// p
							agentSpec.setValue(ATTSPEC, accSpec);
							State [] sList = {ss};
							return sList;
						}
					}
				}
				else{
					System.out.println("[performActionHelper] Error: invalid formula.");

				}
			}
			State [] sList = null;
			return sList;
		}

		// Wrap up with the transition:
		// get the agent,
		// read its x and y position,
		// randomly select a direction,
		// take the corresponding transition,
		// set the x and y position.
		@Override
		protected State performActionHelper(State s, String[] params) {

			//wm
			// consider transitions of the specification MDP part
			State sCopy = s.copy();
			List <TransitionProbability> tps = this.getTransitions(s, "");
			double sum = 0.;
			double r = Math.random();
			for(TransitionProbability tp : tps){
				sum += tp.p;
				if(r < sum){
					sCopy = tp.s.copy();
				}
			}
			//return the state we just modified
			return sCopy;
		}

		// Take a deterministic transition from position (curX, curY) by taking
		// action direction.
		public int[] moveResult(int curX, int curY, int direction) {
			//first get change in x and y from direction using 0: north; 1: south; 2:east; 3: west
			int xdelta = 0;
			int ydelta = 0;
			if(direction == 0){
				ydelta = 1;
			}
			else if(direction == 1){
				ydelta = -1;
			}
			else if(direction == 2){
				xdelta = 1;
			}
			else{
				xdelta = -1;
			}

			int nx = curX + xdelta;
			int ny = curY + ydelta;

			int width = EnvironmentMDP.this.map.length;
			int height = EnvironmentMDP.this.map[0].length;

			// Avoid running out of the boundary or running into the walls
			if (nx<0 || nx>=width || ny<0 || ny>=height ||
					EnvironmentMDP.this.map[nx][ny]==1){
				nx = curX;
				ny = curY;
			}
			return new int[]{nx, ny};
		}

		// get possible outcomes
		@Override
		public List<TransitionProbability> getTransitions(State s, String [] params){

			//get agent and current position
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getIntValForAttribute(ATTX);
			int curY = agent.getIntValForAttribute(ATTY);

			// wm
			// get spec information
			ObjectInstance agentSpec = s.getFirstObjectOfClass(CLASSSPEC);
			int curSpec = agentSpec.getIntValForAttribute(ATTSPEC);

			List<TransitionProbability> tps = new ArrayList<TransitionProbability>(4);
			TransitionProbability noChangeTransition = null;

			for(int i = 0; i < this.directionProbs.length; i++){
				int [] newPos = this.moveResult(curX, curY, i);

				//wm
				State [] newStateList = SpecMoveResult(s, newPos);

				for (int j = 0; j < newStateList.length; j++){
					State newState = newStateList[j];
					ObjectInstance newAgentSpec = newState.getFirstObjectOfClass(CLASSSPEC);
					int newSpec = newAgentSpec.getIntValForAttribute(ATTSPEC);

					// They only consider the successors they have already seen
					if(newPos[0] != curX || newPos[1] != curY || newSpec != curSpec){
						//new possible outcome
						State ns = newState.copy();
						ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
						nagent.setValue(ATTX, newPos[0]);
						nagent.setValue(ATTY, newPos[1]);

						ObjectInstance nspec = ns.getFirstObjectOfClass(CLASSSPEC);
						nspec.setValue(ATTSPEC, newSpec);

						// wm
						// modify the probability by specification MDP transition probability
						double prob = SpecMoveDistribution(s, ns);
						if (prob < 0){
							System.out.println("-----------------------------------------------");
							System.out.println("curX = "+curX+", curY = "+curY+", newX = "+newPos[0]+", newY = "+newPos[1]);
							System.out.println("curSpec = "+curSpec+", newSpec = "+newSpec+", prob = "+prob);
							System.out.println("[getTransitions] Error: negative probability. ");
						}

						//create transition probability object and add to our list of outcomes
						tps.add(new TransitionProbability(ns, this.directionProbs[i] * prob));
					}
					else{
						//this direction didn't lead anywhere new
						//if there are existing possible directions
						//that wouldn't lead anywhere, aggregate with them
						// e.g. in the lower left corner, moving left and moving down is the same, and the prob of
						// staying at the original position should be increased.
						if(noChangeTransition != null){
							// wm
							// modify the probability by specification MDP transition probability
							double prob = SpecMoveDistribution(s, s);
							if (prob < 0){
								System.out.println("[getTransitions] Error: negative probability. ");
							}

							noChangeTransition.p += this.directionProbs[i] * prob;
						}
						else{
							// wm
							// modify the probability by specification MDP transition probability
							double prob = SpecMoveDistribution(s, s);
							if (prob < 0){
								System.out.println("[getTransitions] Error: negative probability. ");
							}

							//otherwise create this new state and transition
							noChangeTransition = new TransitionProbability(s.copy(),
									this.directionProbs[i] * prob);
							tps.add(noChangeTransition);
						}
					}
				}
			}
			return tps;
		}

	}


	protected class AtLocation extends PropositionalFunction {

		public AtLocation(Domain domain){
			super(PFAT, domain, new String []{CLASSAGENT, CLASSSPEC});
		}

		// Return true if the agent has reached the goal position
		@Override
		public boolean isTrue(State s, String[] params) {

			ObjectInstance spec = s.getFirstObjectOfClass(CLASSSPEC);
			int sState = spec.getIntValForAttribute(ATTSPEC);
			int tState = spec.getIntValForAttribute(ATTSPECTRUE);
			int fState = spec.getIntValForAttribute(ATTSPECFALSE);

			return ((sState == tState) || (sState == fState));

		}


	}


	// Create and initialize a new state
	//-- A state contains
	//-- an agent object instance,
	//-- a location object instance,
	//-- a specification object instance.
	public static State getNewState(Domain domain, int[] initPos, int indFormula, int [] paramFormula, int specTrue, int specFalse) {

		State s = new State();
		ObjectInstance agent = new ObjectInstance(domain.getObjectClass(CLASSAGENT), "agent0");
		agent.setValue(ATTX, initPos[0]);
		agent.setValue(ATTY, initPos[1]);

		// wm
		ObjectInstance specification = new ObjectInstance(domain.getObjectClass(CLASSSPEC), "spec0");
		specification.setValue(ATTSPEC, 0);
		specification.setValue(ATTSPECTRUE, specTrue);
		specification.setValue(ATTSPECFALSE, specFalse);
		specification.setValue(ATTFORMULA, indFormula);
		specification.setValue(ATTPARAM, paramFormula);

		s.addObject(agent);
		// wm
		s.addObject(specification);

		return s;

	}

	//-- Visualizer --> StateRenderLayer --> StaticPainter
	public Visualizer getVisualizer() {

		return new Visualizer(this.getStateRenderLayer());
	}

	public StateRenderLayer getStateRenderLayer() {

		StateRenderLayer rl = new StateRenderLayer();
		rl.addStaticPainter(new WallPainter());
		rl.addObjectClassPainter(CLASSAGENT, new AgentPainter());


		return rl;

	}

	public class LocationPainter implements ObjectPainter {

		@Override
		public void paintObject(Graphics2D g2, State s, ObjectInstance ob,
								float cWidth, float cHeight) {

			//agent will be filled in blue
			g2.setColor(Color.BLUE);

			//set up floats for the width and height of our domain
			float fWidth = EnvironmentMDP.this.map.length;
			float fHeight = EnvironmentMDP.this.map[0].length;

			//determine the width of a single cell on our canvas
			//such that the whole map can be painted
			float width = cWidth / fWidth;
			float height = cHeight / fHeight;

			int ax = ob.getIntValForAttribute(ATTX);
			int ay = ob.getIntValForAttribute(ATTY);

			//left coordinate of cell on our canvas
			float rx = ax*width;

			//top coordinate of cell on our canvas
			//coordinate system adjustment because the java canvas
			//origin is in the top left instead of the bottom right
			float ry = cHeight - height - ay*height;

			//paint the rectangle
			g2.fill(new Rectangle2D.Float(rx, ry, width, height));
		}
	}

	public class AgentPainter implements ObjectPainter{

		@Override
		public void paintObject(Graphics2D g2, State s, ObjectInstance ob,
								float cWidth, float cHeight) {

			//agent will be filled in gray
			g2.setColor(Color.GRAY);

			//set up floats for the width and height of our domain
			float fWidth = EnvironmentMDP.this.map.length;
			float fHeight = EnvironmentMDP.this.map[0].length;

			//determine the width of a single cell
			//on our canvas such that the whole map can be painted
			float width = cWidth / fWidth;
			float height = cHeight / fHeight;

			int ax = ob.getIntValForAttribute(ATTX);
			int ay = ob.getIntValForAttribute(ATTY);

			float rx = ax*width;
			float ry = cHeight - height - ay*height;

			//paint the rectangle
			g2.fill(new Ellipse2D.Float(rx, ry, width, height));
		}

	}

	// Paint the wall blocks
	public class WallPainter implements StaticPainter {

		@Override
		public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {

			//walls will be filled in black
			g2.setColor(Color.BLACK);

			//set up floats for the width and height of our domain
			float fWidth = EnvironmentMDP.this.map.length;
			float fHeight = EnvironmentMDP.this.map[0].length;

			//determine the width of a single cell
			//on our canvas such that the whole map can be painted
			float width = cWidth / fWidth;
			float height = cHeight / fHeight;

			for (int i = 0; i < fWidth; i++){
				for (int j = 0; j < fHeight; j++){
					if (EnvironmentMDP.this.map[i][j]==1){
						//left coordinate of cell on our canvas
						float rx = i*width;
						//top coordinate of cell on our canvas
						//coordinate system adjustment because the java canvas
						//origin is in the top left instead of the bottom right
						float ry = cHeight - height - j*height;
						//paint the rectangle
						g2.fill(new Rectangle2D.Float(rx, ry, width, height));
					}
				}
			}
		}
	}

	public static class Reward implements RewardFunction {

		int specTrue;
		int specFalse;

		public Reward(int specT, int specF){
			this.specTrue = specT;
			this.specFalse = specF;
		}


		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			ObjectInstance spec = sprime.getFirstObjectOfClass(CLASSSPEC);
			int aSpec = spec.getIntValForAttribute(ATTSPEC);
			int tSpec = spec.getIntValForAttribute(ATTSPECTRUE);
			int fSpec = spec.getIntValForAttribute(ATTSPECFALSE);

			ObjectInstance oldSpec = s.getFirstObjectOfClass(CLASSSPEC);
			int oSpec = oldSpec.getIntValForAttribute(ATTSPEC);

			if (aSpec == tSpec && !(oSpec == tSpec)){
				return 10;
			}
			else if (aSpec == fSpec){
				return 0;
			}
			else{
				return 0;
			}

		}

	}

	public static class AbsorbingState implements TerminalFunction{

		@Override
		public boolean isTerminal(State s) {

			// wm
			ObjectInstance spec = s.getFirstObjectOfClass(CLASSSPEC);
			int as = spec.getIntValForAttribute(ATTSPEC);
			int specTrue = spec.getIntValForAttribute(ATTSPECTRUE);
			int specFalse = spec.getIntValForAttribute(ATTSPECFALSE);

			return ((as == specTrue) || (as == specFalse));
		}
	}

	// wm
	public static Domain SpecificMDP(Domain domain, int formulaIndex){
		Attribute specAtt = new Attribute(domain, ATTSPEC, AttributeType.INT);
		if (formulaIndex == 1 || formulaIndex == 2){
			specAtt.setLims(0, 2);
		}
		else if (formulaIndex == 3 || formulaIndex == 4){
			specAtt.setLims(0, 3);
		}
		else{
			System.out.println("[ERROR] Invalid input formulaIndex" + formulaIndex);
		}
		Attribute specTrueAtt = new Attribute(domain, ATTSPECTRUE, AttributeType.INT);
		Attribute specFalseAtt = new Attribute(domain, ATTSPECFALSE, AttributeType.INT);
		Attribute specFormulaAtt = new Attribute(domain, ATTFORMULA, AttributeType.INT);
		Attribute specParamAtt = new Attribute(domain, ATTPARAM, AttributeType.INTARRAY);

		ObjectClass specClass = new ObjectClass(domain, CLASSSPEC);
		specClass.addAttribute(specAtt);
		specClass.addAttribute(specTrueAtt);
		specClass.addAttribute(specFalseAtt);
		specClass.addAttribute(specFormulaAtt);
		specClass.addAttribute(specParamAtt);

		return domain;
	}

	public int getSafetyMap(int xPos, int yPos){
		int row = this.safetyMap.length;
		int col = this.safetyMap[0].length;
		if(row <= xPos && col <= yPos){
			System.out.println("Positions should be from [0,0] to ["+row+","+col+"]!");
			return -1;
		}
		else{
			return this.safetyMap[xPos][yPos];
		}
	}

	public boolean isP(int xPos, int yPos){
		// targetP is the value of labels for positions where p holds.
		int targetP = 2;

		return (getSafetyMap(xPos, yPos)==targetP);
	}

	public static void main(String [] args){

		originalMain(args);

	}

	public static void testTransitionsMain(String [] args){
		EnvironmentMDP gen = new EnvironmentMDP();

		// Specify the type of the GLTL formula
		int numFormula = 4;
		int [] specParam = {0,0};
		// Specify the corresponding parameters
		if (numFormula == 1 || numFormula == 2){
			// 1. F k p
			// 2. G k p
			int paramK = 10;
			specParam[0] = paramK;
		}
		else if (numFormula == 3 || numFormula == 4){
			// 3. F l (G k p)
			// 4. G l (F k p)
			int paramK = 10;
			int paramL = 10;
			specParam[0] = paramK;
			specParam[1] = paramL;
		}
		// Specify the initial position here
		int [] initPos = {10,10};

		Domain domain = gen.generateDomain();


		domain = SpecificMDP(domain, numFormula);
		int specTrue = 1;
		int specFalse = 2;
		gen.tf = new AbsorbingState();
		gen.rf = new Reward(specTrue, specFalse);

		System.out.println("Current Formula: "+numFormula);
		State initialState = EnvironmentMDP.getNewState(domain, initPos, numFormula, specParam, specTrue, specFalse);

		Action north = domain.getAction(ACTIONNORTH);
		List<TransitionProbability> tps = north.getTransitions(initialState, "");
		double sum = 0.;
		for(TransitionProbability tp : tps){
			System.out.println(tp.p + ": " + tp.s.toString() + "\n-----");
			sum += tp.p;
		}

		System.out.println("=========\nsum = " + sum);
	}

	public static void originalMain(String [] args){
		EnvironmentMDP gen = new EnvironmentMDP();

		// Specify the type of the GLTL formula
		int numFormula = 2;
		int [] specParam = {0,0};
		// Specify the corresponding parameters
		if (numFormula == 1 || numFormula == 2){
			// 1. F k p
			// 2. G k p
			int paramK = 10;
			specParam[0] = paramK;
		}
		else if (numFormula == 3 || numFormula == 4){
			// 3. F l (G k p)
			// 4. G l (F k p)
			int paramK = 10;
			int paramL = 10;
			specParam[0] = paramK;
			specParam[1] = paramL;
		}
		// Specify the initial position here
		int [] initPos = {10,10};

		Domain domain = gen.generateDomain();

		// wm
		domain = SpecificMDP(domain, numFormula);
		int specTrue = 1;
		int specFalse = 2;
		gen.tf = new AbsorbingState();
		gen.rf = new Reward(specTrue, specFalse);

		System.out.println("Current Formula: "+numFormula);
		State initialState = EnvironmentMDP.getNewState(domain, initPos, numFormula, specParam, specTrue, specFalse);

		//set up the state hashing system
		gen.hashingFactory.setAttributesForClass(EnvironmentMDP.CLASSAGENT,
				domain.getObjectClass(EnvironmentMDP.CLASSAGENT).attributeList);
		gen.hashingFactory.setAttributesForClass(EnvironmentMDP.CLASSSPEC,
				domain.getObjectClass(EnvironmentMDP.CLASSSPEC).attributeList);

		// empty figure
		VisualActionObserver observer = new VisualActionObserver(domain, gen.getVisualizer());
		(gen.domain).setActionObserverForAllAction(observer);
		observer.initGUI();

		String outputPath = "output/";

		gen.ValueIteration(outputPath, initialState);

		//create the state parser
		gen.sp = new GridWorldStateParser(domain);
	}


	public void ValueIteration(String outputPath, State initialState) {

		if(!outputPath.endsWith("/")){
			outputPath = outputPath + "/";
		}

		OOMDPPlanner planner = new ValueIteration(domain, rf, tf, 0.99, hashingFactory,
				0.0001, (int) Math.pow(10, 7)); // 0.001, 100
		planner.planFromState(initialState);

		//create a Q-greedy policy from the planner
		Policy p = new GreedyQPolicy((QComputablePlanner)planner);

		//record the plan results to a file
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "planResult", sp);

		//visualize the value function and policy
		this.valueFunctionVisualize((QComputablePlanner)planner, p, initialState);

	}

	public void valueFunctionVisualize(QComputablePlanner planner, Policy p, State initialState){
		List <State> allStates = StateReachability.getReachableStates(initialState,
				(SADomain) domain, hashingFactory);

		//wm
		System.out.println("Size of allStates: " + allStates.size());
		List <State> allStatesNonTerminal = new ArrayList<State>(allStates);
		Iterator<State> itr = allStatesNonTerminal.iterator();
		int tCnt = 0;
		int fCnt = 0;
		int extraCnt = 0;
		while (itr.hasNext()){
			State ss = itr.next();
			ObjectInstance spec = ss.getFirstObjectOfClass(CLASSSPEC);
			int state = spec.getIntValForAttribute(ATTSPEC);
			int tState = spec.getIntValForAttribute(ATTSPECTRUE);
			int fState = spec.getIntValForAttribute(ATTSPECFALSE);
			if (state == tState){
				tCnt = tCnt + 1;
				itr.remove();
			}
			else if (state == fState){
				fCnt = fCnt + 1;
				itr.remove();
			}
			// plot only the states with initial state in the spec MDP
			else if (! (state == 0)){
				extraCnt = extraCnt + 1;
				itr.remove();
			}
		}
		System.out.println("Size of allStatesNonTerminal: " + allStatesNonTerminal.size());
		System.out.println("tCnt = "+tCnt+"; fCnt = "+fCnt+"; extraCnt = "+extraCnt);

		LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
		rb.addNextLandMark(0., Color.RED);
		rb.addNextLandMark(1., Color.BLUE);

		StateValuePainter2D svp = new StateValuePainter2D(rb);
		svp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX,
				GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);


		PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
		spp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX,
				GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONNORTH, new ArrowActionGlyph(0));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONSOUTH, new ArrowActionGlyph(1));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONEAST, new ArrowActionGlyph(2));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONWEST, new ArrowActionGlyph(3));
		spp.setRenderStyle(PolicyGlyphPainter2D.PolicyGlyphRenderStyle.DISTSCALED);

		// wm
		ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStatesNonTerminal, svp, planner);
		//ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, planner);
		gui.setSpp(spp);
		gui.setPolicy(p);
		gui.setBgColor(Color.GRAY);
		gui.initGUI();
	}

}
