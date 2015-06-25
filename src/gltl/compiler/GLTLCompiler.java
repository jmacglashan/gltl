package gltl.compiler;

import burlap.debugtools.RandomFactory;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * @author James MacGlashan.
 */
public class GLTLCompiler implements DomainGenerator{


	public static final String 					ATTSPEC = "##spec";
	public static final String 					ATTSPECTRUE = "##specTrue";
	public static final String					ATTSPECFALSE = "##specFalse";
	public static final String 					ATTFORMULA = "##specformula";
	public static final String 					ATTPARAM = "##specParameter";

	public static final String 					CLASSSPEC = "##spec";


	protected Domain 							environmentDomain;
	protected String formula;
	protected SymbolEvaluator					symbolEvaluator;


	public GLTLCompiler(String formula, Map<String, PropositionalFunction> symbolMap, Domain environmentDomain){
		this.formula = formula;
		this.symbolEvaluator = new SymbolEvaluator(symbolMap);
		this.environmentDomain = environmentDomain;
	}

	public String getFormula(){
		return this.formula;
	}

	public void setFormula(String formula, Map<String, PropositionalFunction> symbolMap){
		this.formula = formula;
		this.symbolEvaluator = new SymbolEvaluator(symbolMap);
	}

	public Domain getEnvironmentDomain() {
		return environmentDomain;
	}

	public void setEnvironmentDomain(Domain environmentDomain) {
		this.environmentDomain = environmentDomain;
	}

	public State addInitialTaskStateToEnvironmentState(Domain compiledDomain, State environmentState){

		State newState = environmentState.copy();

		//first remove any task spec objects if they are there
		for(ObjectInstance ob : newState.getObjectsOfClass(CLASSSPEC)){
			newState.removeObject(ob.getName());
		}

		//now add the appropriate initial state
		int numFormula = Integer.parseInt(this.formula);
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

		ObjectInstance taskOb = new ObjectInstance(compiledDomain.getObjectClass(CLASSSPEC), CLASSSPEC);
		taskOb.setValue(ATTFORMULA, numFormula);
		taskOb.setValue(ATTPARAM, specParam);
		taskOb.setValue(ATTSPECTRUE, 1);
		taskOb.setValue(ATTSPECFALSE, 2);
		taskOb.setValue(ATTSPEC, 0);

		newState.addObject(taskOb);

		return newState;

	}

	@Override
	public Domain generateDomain() {


		Domain domain = new SADomain();
		int formulaIndex = Integer.parseInt(this.formula);

		Attribute specAtt = new Attribute(domain, ATTSPEC, Attribute.AttributeType.INT);
		if (formulaIndex == 1 || formulaIndex == 2){
			specAtt.setLims(0, 2);
		}
		else if (formulaIndex == 3 || formulaIndex == 4){
			specAtt.setLims(0, 3);
		}
		else{
			System.out.println("[ERROR] Invalid input formulaIndex" + formulaIndex);
		}
		Attribute specTrueAtt = new Attribute(domain, ATTSPECTRUE, Attribute.AttributeType.INT);
		Attribute specFalseAtt = new Attribute(domain, ATTSPECFALSE, Attribute.AttributeType.INT);
		Attribute specFormulaAtt = new Attribute(domain, ATTFORMULA, Attribute.AttributeType.INT);
		Attribute specParamAtt = new Attribute(domain, ATTPARAM, Attribute.AttributeType.INTARRAY);

		ObjectClass specClass = new ObjectClass(domain, CLASSSPEC);
		specClass.addAttribute(specAtt);
		specClass.addAttribute(specTrueAtt);
		specClass.addAttribute(specFalseAtt);
		specClass.addAttribute(specFormulaAtt);
		specClass.addAttribute(specParamAtt);

		for(Action a : this.environmentDomain.getActions()){
			new CompiledAction(a, domain, this.formula, this.symbolEvaluator);
		}

		return domain;

	}

	public RewardFunction generateRewardFunction(){

		return new RewardFunction() {
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
		};

	}

	public TerminalFunction generateTerminalFunction(){
		return new TerminalFunction() {
			@Override
			public boolean isTerminal(State s) {
				ObjectInstance spec = s.getFirstObjectOfClass(CLASSSPEC);
				int as = spec.getIntValForAttribute(ATTSPEC);
				int specTrue = spec.getIntValForAttribute(ATTSPECTRUE);
				int specFalse = spec.getIntValForAttribute(ATTSPECFALSE);

				return ((as == specTrue) || (as == specFalse));
			}
		};
	}


	public static class CompiledAction extends Action {

		protected Action				srcAction;
		protected String 				formula;
		protected SymbolEvaluator 		symbolEvaluator;


		public CompiledAction(Action srcAction, Domain domain, String formula, SymbolEvaluator symbolEvaluator){
			super(srcAction.getName(), domain, srcAction.getParameterClasses(), srcAction.getParameterOrderGroups());
			this.srcAction = srcAction;
			this.formula = formula;
			this.symbolEvaluator = symbolEvaluator;
		}

		@Override
		public List<TransitionProbability> getTransitions(State s, String[] params) {

			//get the environment mdp transition dynamics
			List <TransitionProbability> environmentTPs = this.srcAction.getTransitions(s, params);

			//reserve space for the joint task-environment mdp transitions
			List <TransitionProbability> jointTPs = new ArrayList<TransitionProbability>(environmentTPs.size()*2);

			//perform outer loop of transitions cross product over environment transitions
			for(TransitionProbability etp : environmentTPs){

				//get the task transitions and expand them with the environment transition
				List<TaskMPDTransition> taskTPs = this.getTaskTransitions(s, etp.s);
				double taskSum = 0.;
				for(TaskMPDTransition ttp : taskTPs){
					State ns = etp.s.copy();
					//remove the old task spec
					ns.removeObject(ns.getFirstObjectOfClass(CLASSSPEC).getName());
					//set the new task spec
					ns.addObject(ttp.taskObject);
					double p = etp.p*ttp.p;
					TransitionProbability jtp = new TransitionProbability(ns, p);
					jointTPs.add(jtp);

					taskSum += ttp.p;
				}

				if(Math.abs(taskSum-1.) > 1e-5){
					throw new RuntimeException("Error, could not return transition probabilities because task MDP transition probabilities summed to " + taskSum + " instead of 1.");
				}

			}


			return jointTPs;
		}



		protected List<TaskMPDTransition> getTaskTransitions(State s, State nextEnvState){

			State ss = s.copy();

			ObjectInstance agentSpec = s.getFirstObjectOfClass(CLASSSPEC);

			int curStateSpec = agentSpec.getIntValForAttribute(ATTSPEC);
			int accSpec = agentSpec.getIntValForAttribute(ATTSPECTRUE);
			int rejSpec = agentSpec.getIntValForAttribute(ATTSPECFALSE);
			int formulaSpec = agentSpec.getIntValForAttribute(ATTFORMULA);
			int [] paramSpec = agentSpec.getIntArrayValForAttribute(ATTPARAM);

			boolean p = this.symbolEvaluator.eval("P", nextEnvState);

			List <TaskMPDTransition> transitions = new ArrayList<TaskMPDTransition>();

			if (curStateSpec == accSpec || curStateSpec == rejSpec){
				transitions.add(new TaskMPDTransition(agentSpec.copy(), 1.));
				return transitions;
			}
			else{
				// get the goal position, which is initialized in getNewState
				// Neither accepted nor rejected yet
				if (formulaSpec == 1){
					// F k p
					if (p){
						// p
						ObjectInstance nagentSpec = agentSpec.copy();
						nagentSpec.setValue(ATTSPEC, accSpec);
						transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.));
						return transitions;
					}
					else{
						// not p
						transitions.add(new TaskMPDTransition(agentSpec.copy(), (paramSpec[0] - 1.0) / (double)paramSpec[0]));
						ObjectInstance nagentSpec = agentSpec.copy();
						nagentSpec.setValue(ATTSPEC, rejSpec);
						transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.0 / (double) paramSpec[0]));
						return transitions;

//						State ss2 = ss.copy();
//						agentSpec.setValue(ATTSPEC, rejSpec);
//						State [] sList = {ss, ss2};
//						//return sList;
					}
				}
				else if (formulaSpec == 2) {
					// G k p
					if (! p){
						// not p
						ObjectInstance nagentSpec = agentSpec.copy();
						nagentSpec.setValue(ATTSPEC, rejSpec);
						transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.));
						return transitions;

//						agentSpec.setValue(ATTSPEC, rejSpec);
//						State [] sList = {ss};
//						//return sList;
					}
					else{
						// p
						transitions.add(new TaskMPDTransition(agentSpec.copy(), (paramSpec[0] - 1.0) / (double)paramSpec[0]));
						ObjectInstance nagentSpec = agentSpec.copy();
						nagentSpec.setValue(ATTSPEC, accSpec);
						transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.0 / (double) paramSpec[0]));
						return transitions;

//						State ss2 = ss.copy();
//						agentSpec.setValue(ATTSPEC, accSpec);
//						State [] sList = {ss, ss2};
//						//return sList;
					}
				}
				else if (formulaSpec == 3){
					// F l (G k p)

					double k = (double) paramSpec[0];
					double l = (double) paramSpec[1];

					if (curStateSpec == 0){
						// still at the initial state

						if (p){
							// p
							ObjectInstance nagentSpec = agentSpec.copy();
							transitions.add(new TaskMPDTransition(nagentSpec.copy(), (k-1)*(l-1)/k/l));

							ObjectInstance n2agentSpec = agentSpec.copy();
							n2agentSpec.setValue(ATTSPEC, 3);
							transitions.add(new TaskMPDTransition(n2agentSpec.copy(), (k-1)/k/l));

							ObjectInstance n3agentSpec = agentSpec.copy();
							n3agentSpec.setValue(ATTSPEC, accSpec);
							transitions.add(new TaskMPDTransition(n3agentSpec.copy(), 1/k));
							return transitions;

//							State ss2 = ss.copy();
//							agentSpec.setValue(ATTSPEC, 3);
//							State ss3 = ss.copy();
//							agentSpec.setValue(ATTSPEC, accSpec);
//							State[] sList = {ss, ss2, ss3};
//							//return sList;
						}
						else{
							// not p
							transitions.add(new TaskMPDTransition(agentSpec.copy(), 1.0 - 1.0 / l));

							ObjectInstance nagentSpec = agentSpec.copy();
							nagentSpec.setValue(ATTSPEC, rejSpec);
							transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.0 / l));
							return transitions;

//							State ss2 = ss.copy();
//							agentSpec.setValue(ATTSPEC, rejSpec);
//							State [] sList = {ss, ss2};
//							//return sList;
						}
					}
					else if (curStateSpec == 3){
						// the second non-terminating state, which is the last chance to satisfy the spec
						if (p){
							// p
							transitions.add(new TaskMPDTransition(agentSpec.copy(), 1.0 - 1.0 / k));

							ObjectInstance nagentSpec = agentSpec.copy();
							nagentSpec.setValue(ATTSPEC, accSpec);
							transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.0 / k));
							return transitions;

//							State ss2 = ss.copy();
//							agentSpec.setValue(ATTSPEC, accSpec);
//							State [] sList = {ss, ss2};
//							//return sList;
						}
						else{
							// not p
							ObjectInstance nagentSpec = agentSpec.copy();
							nagentSpec.setValue(ATTSPEC, rejSpec);
							transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.0));
							return transitions;

//							agentSpec.setValue(ATTSPEC, rejSpec);
//							State [] sList = {ss};
//							//return sList;
						}
					}
				}
				else if (formulaSpec == 4){
					// G l (F k p)

					double k = (double) paramSpec[0];
					double l = (double) paramSpec[1];

					if (curStateSpec == 0){
						// still at the initial state
						if (! p){
							// not p
							ObjectInstance nagentSpec = agentSpec.copy();
							transitions.add(new TaskMPDTransition(nagentSpec.copy(), (k-1)*(l-1)/k/l));

							ObjectInstance n2agentSpec = agentSpec.copy();
							n2agentSpec.setValue(ATTSPEC, 3);
							transitions.add(new TaskMPDTransition(n2agentSpec.copy(), (l-1)/k/l));

							ObjectInstance n3agentSpec = agentSpec.copy();
							n3agentSpec.setValue(ATTSPEC, rejSpec);
							transitions.add(new TaskMPDTransition(n3agentSpec.copy(), 1/l));
							return transitions;

//							State ss2 = ss.copy();
//							agentSpec.setValue(ATTSPEC, 3);
//							State ss3 = ss.copy();
//							agentSpec.setValue(ATTSPEC, rejSpec);
//							State[] sList = {ss, ss2, ss3};
//							//return sList;
						}
						else{
							// p
							transitions.add(new TaskMPDTransition(agentSpec.copy(), 1.0 - 1.0 / k));

							ObjectInstance nagentSpec = agentSpec.copy();
							nagentSpec.setValue(ATTSPEC, accSpec);
							transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.0/k));
							return transitions;

//							State ss2 = ss.copy();
//							agentSpec.setValue(ATTSPEC, accSpec);
//							State [] sList = {ss, ss2};
//							//return sList;
						}

					}
					else if (curStateSpec == 3){
						// the second non-terminating state, which is the last chance to satisfy the spec
						if (! p){
							// not p
							transitions.add(new TaskMPDTransition(agentSpec.copy(), 1.0 - 1.0 / l));

							ObjectInstance nagentSpec = agentSpec.copy();
							nagentSpec.setValue(ATTSPEC, rejSpec);
							transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.0 / l));
							return transitions;


//							State ss2 = ss.copy();
//							agentSpec.setValue(ATTSPEC, rejSpec);
//							State [] sList = {ss, ss2};
//							//return sList;
						}
						else{
							// p
							ObjectInstance nagentSpec = agentSpec.copy();
							nagentSpec.setValue(ATTSPEC, accSpec);
							transitions.add(new TaskMPDTransition(nagentSpec.copy(), 1.));
							return transitions;

//							agentSpec.setValue(ATTSPEC, accSpec);
//							State [] sList = {ss};
//							//return sList;
						}
					}
				}
			}


			throw new RuntimeException("Could not define task MDP transitions for formula " + formulaSpec);

		}

		@Override
		protected State performActionHelper(State s, String[] params) {

			//sample an environment state
			State environmentNextState = this.srcAction.performAction(s, params);

			//determine the next task state distribution and sample from it
			List<TaskMPDTransition> taskTPs = this.getTaskTransitions(s, environmentNextState);
			double r = RandomFactory.getMapped(0).nextDouble();
			double sumP = 0.;
			for(TaskMPDTransition ttp : taskTPs){
				sumP += ttp.p;
				if(r < sumP){
					//remove the old task spec
					environmentNextState.removeObject(environmentNextState.getFirstObjectOfClass(CLASSSPEC).getName());
					//set the new task spec
					environmentNextState.addObject(ttp.taskObject);
					return environmentNextState;
				}
			}

			throw new RuntimeException("Could not sample action from " + this.getName() + " because the task transition dynamics did not sum to 1.)");
		}

		@Override
		public boolean parametersAreObjects() {
			return this.srcAction.parametersAreObjects();
		}

		@Override
		public boolean applicableInState(State s, String[] params) {
			return this.srcAction.applicableInState(s, params);
		}



		@Override
		public List<GroundedAction> getAllApplicableGroundedActions(State s) {
			List <GroundedAction> srcGas =  this.srcAction.getAllApplicableGroundedActions(s);
			List <GroundedAction> targetGas = new ArrayList<GroundedAction>(srcGas.size());
			for(GroundedAction ga : srcGas){
				targetGas.add(new GroundedAction(this, ga.params));
			}

			return targetGas;
		}
	}





	public static class SymbolEvaluator{

		protected Map<String, PropositionalFunction> 	symbolMapping;

		public SymbolEvaluator(Map<String, PropositionalFunction> symbolMapping) {
			this.symbolMapping = symbolMapping;
		}

		public Map<String, PropositionalFunction> getSymbolMapping() {
			return symbolMapping;
		}

		public void setSymbolMapping(Map<String, PropositionalFunction> symbolMapping) {
			this.symbolMapping = symbolMapping;
		}

		public boolean eval(String symbol, State s, String...params){
			PropositionalFunction pf = this.symbolMapping.get(symbol);
			if(pf == null){
				throw new RuntimeException("Symbol " + symbol + " cannot be evaluated because it is undefined");
			}
			return pf.isTrue(s, params);
		}
	}


	protected static class TaskMPDTransition{

		public ObjectInstance taskObject;
		public double p;

		public TaskMPDTransition(ObjectInstance taskObject, double p){
			this.taskObject = taskObject;
			this.p = p;
		}

	}

}
