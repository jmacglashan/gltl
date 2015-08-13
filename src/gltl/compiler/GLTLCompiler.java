package gltl.compiler;

import burlap.debugtools.RandomFactory;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import com.sun.javafx.tk.Toolkit;

import java.util.*;

/**
 * @author James MacGlashan, reworked by Michael Littman
 */
public class GLTLCompiler implements DomainGenerator {

	public static final String ATTSPEC = "##spec";
	public static final String ATTSPECTRUE = "##specTrue";
	public static final String ATTSPECFALSE = "##specFalse";
	public static final String ATTFORMULA = "##specformula";
	public static final String ATTPARAM = "##specParameter";

	public static final String CLASSSPEC = "##spec";


	protected Domain environmentDomain;
	protected String formula;
	protected TransitionQuery transitionQuery;
	protected SymbolEvaluator symbolEvaluator;


	public GLTLCompiler(String formula, Map<String, PropositionalFunction> symbolMap, Domain environmentDomain) {
		this.formula = formula;
		this.symbolEvaluator = new SymbolEvaluator(symbolMap);
		this.environmentDomain = environmentDomain;
	}

	public String getFormula() {
		return this.formula;
	}

	public void setFormula(String formula, Map<String, PropositionalFunction> symbolMap) {
		this.formula = formula;
		this.symbolEvaluator = new SymbolEvaluator(symbolMap);
	}

	public Domain getEnvironmentDomain() {
		return environmentDomain;
	}

	public void setEnvironmentDomain(Domain environmentDomain) {
		this.environmentDomain = environmentDomain;
	}

	public State addInitialTaskStateToEnvironmentState(Domain compiledDomain, State environmentState) {

		State newState = environmentState.copy();

		//first remove any task spec objects if they are there
		for (ObjectInstance ob : newState.getObjectsOfClass(CLASSSPEC)) {
			newState.removeObject(ob.getName());
		}

		ObjectInstance taskOb = new ObjectInstance(compiledDomain.getObjectClass(CLASSSPEC), CLASSSPEC);
		// taskOb.setValue(ATTFORMULA, this.formula);
		// taskOb.setValue(ATTPARAM, specParam);
		// taskOb.setValue(ATTSPECTRUE, 1);
		// taskOb.setValue(ATTSPECFALSE, 2);
		taskOb.setValue(ATTSPEC, 2);

		newState.addObject(taskOb);

		return newState;

	}

	@Override
	public Domain generateDomain() {


		Domain domain = new SADomain();
//		int formulaIndex = Integer.parseInt(this.formula);
//
//		Attribute specAtt = new Attribute(domain, ATTSPEC, Attribute.AttributeType.INT);
//		if (formulaIndex == 1 || formulaIndex == 2){
//			specAtt.setLims(0, 2);
//		}
//		else if (formulaIndex == 3 || formulaIndex == 4){
//			specAtt.setLims(0, 3);
//		}
//		else{
//			System.out.println("[ERROR] Invalid input formulaIndex" + formulaIndex);
//		}
		//Attribute specTrueAtt = new Attribute(domain, ATTSPECTRUE, Attribute.AttributeType.INT);
		//Attribute specFalseAtt = new Attribute(domain, ATTSPECFALSE, Attribute.AttributeType.INT);
		//Attribute specFormulaAtt = new Attribute(domain, ATTFORMULA, Attribute.AttributeType.STRING);
		Attribute specAtt = new Attribute(domain, ATTSPEC, Attribute.AttributeType.INT);
		// Attribute specParamAtt = new Attribute(domain, ATTPARAM, Attribute.AttributeType.INTARRAY);

		ObjectClass specClass = new ObjectClass(domain, CLASSSPEC);
		specClass.addAttribute(specAtt);
		//specClass.addAttribute(specTrueAtt);
		//specClass.addAttribute(specFalseAtt);
		//specClass.addAttribute(specFormulaAtt);
		// specClass.addAttribute(specParamAtt);

		TransitionQuery transitionQuery = TransitionQuery.compileFormula(domain, this.formula, 0);

//		TransitionQuery transitionQuery = new TransitionQuery(domain, "Q");
//		// System.out.println(transitionQuery.symbolMap.size());
//		transitionQuery = transitionQuery.eventually(.99);
//		transitionQuery = transitionQuery.not();
//		transitionQuery = transitionQuery.eventually(.99);
//		transitionQuery = transitionQuery.not();

//      DUMP GRAPH for debugging
		System.out.println(transitionQuery.symbolMap);
		for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e : transitionQuery.transitions.entrySet()) {
			System.out.println(e.getKey().a + ", " + e.getKey().b);
			for (TaskMDPTransition trans : e.getValue()) {
				System.out.println(trans.p + "->" + trans.taskObject.getIntValForAttribute(ATTSPEC));
			}
		}
		System.out.println("=========");

		for (Action a : this.environmentDomain.getActions()) {
			new CompiledAction(a, domain, this.formula, this.symbolEvaluator, transitionQuery);
		}

		return domain;

	}

	public RewardFunction generateRewardFunction() {

		return new RewardFunction() {
			@Override
			public double reward(State s, GroundedAction a, State sprime) {
				ObjectInstance spec = sprime.getFirstObjectOfClass(CLASSSPEC);
				int aSpec = spec.getIntValForAttribute(ATTSPEC);
//				int tSpec = spec.getIntValForAttribute(ATTSPECTRUE);
//				int fSpec = spec.getIntValForAttribute(ATTSPECFALSE);

				ObjectInstance oldSpec = s.getFirstObjectOfClass(CLASSSPEC);
				int oSpec = oldSpec.getIntValForAttribute(ATTSPEC);

				if (aSpec == 1 && !(oSpec == 1)) {
					return 1.;
				} else if (aSpec == 0) {
					return 0;
				} else {
					return 0;
				}
			}
		};

	}

	public TerminalFunction generateTerminalFunction() {
		return new TerminalFunction() {
			@Override
			public boolean isTerminal(State s) {
				ObjectInstance spec = s.getFirstObjectOfClass(CLASSSPEC);
				int as = spec.getIntValForAttribute(ATTSPEC);
//				int specTrue = spec.getIntValForAttribute(ATTSPECTRUE);
//				int specFalse = spec.getIntValForAttribute(ATTSPECFALSE);

				return ((as == 1) || (as == 0));
			}
		};
	}


	public static class CompiledAction extends Action {

		protected Action srcAction;
		protected String formula;
		protected SymbolEvaluator symbolEvaluator;
		protected TransitionQuery transitions;

		public CompiledAction(Action srcAction, Domain domain, String formula, SymbolEvaluator symbolEvaluator, TransitionQuery transitions) {
			super(srcAction.getName(), domain, srcAction.getParameterClasses(), srcAction.getParameterOrderGroups());
			this.srcAction = srcAction;
			this.formula = formula;
			this.symbolEvaluator = symbolEvaluator;
			this.transitions = transitions;
		}

		@Override
		public List<TransitionProbability> getTransitions(State s, String[] params) {

			//get the environment mdp transition dynamics
			List<TransitionProbability> environmentTPs = this.srcAction.getTransitions(s, params);

			//reserve space for the joint task-environment mdp transitions
			List<TransitionProbability> jointTPs = new ArrayList<TransitionProbability>(environmentTPs.size() * 2);

			//perform outer loop of transitions cross product over environment transitions
			for (TransitionProbability etp : environmentTPs) {

				//get the task transitions and expand them with the environment transition
				List<TaskMDPTransition> taskTPs = this.getTaskTransitions(s, etp.s);
//				System.out.println("===>" + taskTPs.size());
				double taskSum = 0.;
				for (TaskMDPTransition ttp : taskTPs) {
					State ns = etp.s.copy();
					//remove the old task spec
					ns.removeObject(ns.getFirstObjectOfClass(CLASSSPEC).getName());
					//set the new task spec
					ns.addObject(ttp.taskObject);
					double p = etp.p * ttp.p;
					TransitionProbability jtp = new TransitionProbability(ns, p);
					jointTPs.add(jtp);

					taskSum += ttp.p;
				}

				if (Math.abs(taskSum - 1.) > 1e-5) {
					throw new RuntimeException("Error, could not return transition probabilities because task MDP transition probabilities summed to " + taskSum + " instead of 1.");
				}

			}


			return jointTPs;
		}


		// grr, I don't understand Java :-(
		// List<TaskMDPTransition> figureOutFormula(String formula, int pos) {
		//	if (pos >= formula.length) {
		//	}
		// }

// We want to know, given a pair of s and s', what's the probability of making a transition
// from s to s'.

		protected List<TaskMDPTransition> getTaskTransitions(State s, State nextEnvState) {

			State ss = s.copy();

			ObjectInstance agentSpec = s.getFirstObjectOfClass(CLASSSPEC);

			int curStateSpec = agentSpec.getIntValForAttribute(ATTSPEC); // task state (for s)
//			int accSpec = agentSpec.getIntValForAttribute(ATTSPECTRUE);
//			int rejSpec = agentSpec.getIntValForAttribute(ATTSPECFALSE);
//			String formulaSpec = agentSpec.getStringValForAttribute(ATTFORMULA);
			// int [] paramSpec = agentSpec.getIntArrayValForAttribute(ATTPARAM);

			List<String> dependencies = this.transitions.symbolDependencies(curStateSpec);

			int actionlabel = 0;
			for (String symbolName : dependencies) {
				actionlabel = 2 * actionlabel + ((this.symbolEvaluator.eval(symbolName, nextEnvState)) ? 1 : 0);
			}

//			System.out.println(curStateSpec + "/" + actionlabel + ">" + dependencies);
//			System.out.println("::::" + this.transitions.nextTaskStateTransitions(curStateSpec, actionlabel).size());
			return this.transitions.nextTaskStateTransitions(curStateSpec, actionlabel);

//			List <TaskMDPTransition> transitions = new ArrayList<TaskMDPTransition>();
//
//			// Query a data structure that returns task transition objects.
//
//			// Lookup is based on task-state integer, symbol expression.
//			boolean p = this.symbolEvaluator.eval(formulaSpec.substring(2,3), nextEnvState);
//
//
//			// Absorbing state transitions to itself.
//			if (curStateSpec == accSpec || curStateSpec == rejSpec){
//				transitions.add(new TaskMDPTransition(agentSpec.copy(), 1.));
//				return transitions;
//			}
//			else{
//				// get the goal position, which is initialized in getNewState
//				// Neither accepted nor rejected yet
//				if (formulaSpec.charAt(0) == 'F'){
//					boolean p = this.symbolEvaluator.eval(formulaSpec.substring(2,3), nextEnvState);
//					// Fkp
//					int k = (int) (formulaSpec.charAt(1)-'0');
//					double discount = 1.0-Math.pow(.1, (double) k);
//					System.out.println(k + ":" + discount);
//
//					if (p){
//						// p
//						ObjectInstance nagentSpec = agentSpec.copy();
//						nagentSpec.setValue(ATTSPEC, accSpec);
//						transitions.add(new TaskMDPTransition(nagentSpec.copy(), 1.));
//						return transitions;
//					}
//					else{
//						// not p
//						transitions.add(new TaskMDPTransition(agentSpec.copy(), discount));
//						ObjectInstance nagentSpec = agentSpec.copy();
//						nagentSpec.setValue(ATTSPEC, rejSpec);
//						transitions.add(new TaskMDPTransition(nagentSpec.copy(), 1.0-discount));
//						return transitions;
//
////						State ss2 = ss.copy();
////						agentSpec.setValue(ATTSPEC, rejSpec);
////						State [] sList = {ss, ss2};
////						//return sList;
//					}
//				}
//				else if (formulaSpec.charAt(0) == 'U'){
//					// Ukpq
//					boolean p = this.symbolEvaluator.eval(formulaSpec.substring(2,3), nextEnvState);
//					boolean q = this.symbolEvaluator.eval(formulaSpec.substring(3,4), nextEnvState);
//					int k = (int) (formulaSpec.charAt(1)-'0');
//					double discount = 1.0-Math.pow(.1, (double) k);
//
//					if (q){
//						// q => released
//						ObjectInstance nagentSpec = agentSpec.copy();
//						nagentSpec.setValue(ATTSPEC, accSpec);
//						transitions.add(new TaskMDPTransition(nagentSpec.copy(), 1.));
//						return transitions;
//					} else if (!p){
//						// not p => failed
//						ObjectInstance nagentSpec = agentSpec.copy();
//						nagentSpec.setValue(ATTSPEC, rejSpec);
//						transitions.add(new TaskMDPTransition(nagentSpec.copy(), 1.));
//						return transitions;
//
////						agentSpec.setValue(ATTSPEC, rejSpec);
////						State [] sList = {ss};
////						//return sList;
//					}
//					else{
//						// p reject if time runs out
//						transitions.add(new TaskMDPTransition(agentSpec.copy(), discount));
//						ObjectInstance nagentSpec = agentSpec.copy();
//						nagentSpec.setValue(ATTSPEC, rejSpec);
//						transitions.add(new TaskMDPTransition(nagentSpec.copy(), 1.0 - discount));
//						return transitions;
//
////						State ss2 = ss.copy();
////						agentSpec.setValue(ATTSPEC, accSpec);
////						State [] sList = {ss, ss2};
////						//return sList;
//					}
//				}
//				else if (formulaSpec.charAt(0) == 'G'){
//					// Gkp
//					boolean p = this.symbolEvaluator.eval(formulaSpec.substring(2,3), nextEnvState);
//					int k = (int) (formulaSpec.charAt(1)-'0');
//					double discount = 1.0-Math.pow(.1, (double) k);
//
//					if (!p){
//						// not p
//						ObjectInstance nagentSpec = agentSpec.copy();
//						nagentSpec.setValue(ATTSPEC, rejSpec);
//						transitions.add(new TaskMDPTransition(nagentSpec.copy(), 1.));
//						return transitions;
//
////						agentSpec.setValue(ATTSPEC, rejSpec);
////						State [] sList = {ss};
////						//return sList;
//					}
//					else{
//						// p
//						transitions.add(new TaskMDPTransition(agentSpec.copy(), discount));
//						ObjectInstance nagentSpec = agentSpec.copy();
//						nagentSpec.setValue(ATTSPEC, accSpec);
//						transitions.add(new TaskMDPTransition(nagentSpec.copy(), 1.0 - discount));
//						return transitions;
//
////						State ss2 = ss.copy();
////						agentSpec.setValue(ATTSPEC, accSpec);
////						State [] sList = {ss, ss2};
////						//return sList;
//					}
//				}
//			}

			// throw new RuntimeException("Could not define task MDP transitions for formula " + formulaSpec);

		}

		@Override
		protected State performActionHelper(State s, String[] params) {

			//sample an environment state
			State environmentNextState = this.srcAction.performAction(s, params);

			//determine the next task state distribution and sample from it
			List<TaskMDPTransition> taskTPs = this.getTaskTransitions(s, environmentNextState);
			double r = RandomFactory.getMapped(0).nextDouble();
			double sumP = 0.;
			for (TaskMDPTransition ttp : taskTPs) {
				sumP += ttp.p;
				if (r < sumP) {
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
			List<GroundedAction> srcGas = this.srcAction.getAllApplicableGroundedActions(s);
			List<GroundedAction> targetGas = new ArrayList<GroundedAction>(srcGas.size());
			for (GroundedAction ga : srcGas) {
				targetGas.add(new GroundedAction(this, ga.params));
			}

			return targetGas;
		}
	}


	public static class SymbolEvaluator {

		protected Map<String, PropositionalFunction> symbolMapping;

		public SymbolEvaluator(Map<String, PropositionalFunction> symbolMapping) {
			this.symbolMapping = symbolMapping;
		}

		public Map<String, PropositionalFunction> getSymbolMapping() {
			return symbolMapping;
		}

		public void setSymbolMapping(Map<String, PropositionalFunction> symbolMapping) {
			this.symbolMapping = symbolMapping;
		}

		public boolean eval(String symbol, State s, String... params) {
			PropositionalFunction pf = this.symbolMapping.get(symbol);
			if (pf == null) {
				throw new RuntimeException("Symbol " + symbol + " cannot be evaluated because it is undefined");
			}
			return pf.isTrue(s, params);
		}
	}

	protected static class TaskMDPTransition {

		public ObjectInstance taskObject;
		public double p;

		public TaskMDPTransition(ObjectInstance taskObject, double p) {
			this.taskObject = taskObject;
			this.p = p;
		}
	}

	protected static class IntegerPair {
		public int a;
		public int b;

		public IntegerPair(int a, int b) {
			this.a = a;
			this.b = b;
		}

		@Override
		public int hashCode() {
			return a + 31 * b;
		}

		@Override
		public boolean equals(Object obj) {
			IntegerPair p = (IntegerPair) obj;
			return (a == p.a) && (b == p.b);
		}
	}

	// Data structure for holding task transitions.
	public static class TransitionQuery {

		Map<Integer, List<String>> symbolMap = new HashMap<>();
		Map<IntegerPair, List<TaskMDPTransition>> transitions = new HashMap<>();

		public static TransitionQuery compileFormula(Domain domain, String formula, int pos) {
			TransitionQuery subexp;
			TransitionQuery subexp2;
			int k;
			double discount;

			switch (formula.charAt(pos)) {
				case '!':
					subexp= compileFormula(domain, formula, pos + 1);
					return subexp.not();
				case 'F':
					subexp = compileFormula(domain, formula, pos + 2);
					k = (int) (formula.charAt(pos+1)-'0');
					discount = 1.0-Math.pow(.1, (double) k);
					return subexp.eventually(discount);
				case 'G':
					subexp = compileFormula(domain, formula, pos + 2);
					subexp = subexp.not();
					k = (int) (formula.charAt(pos+1)-'0');
					discount = 1.0-Math.pow(.1, (double) k);
					subexp = subexp.eventually(discount);
					return subexp.not();
				case 'U':
					subexp = compileFormula(domain, formula, pos + 2);
					k = (int) (formula.charAt(pos+1)-'0');
					discount = 1.0-Math.pow(.1, (double) k);
					subexp2 = compileFormula(domain, formula, skipFormula(formula, pos + 2));
					return subexp.until(discount, subexp2);
				case '&':
					subexp = compileFormula(domain, formula, pos + 1);
					subexp2 = compileFormula(domain, formula, skipFormula(formula, pos + 1));
					return subexp.and(subexp2);
				case '|':
					subexp = compileFormula(domain, formula, pos + 1);
					subexp2 = compileFormula(domain, formula, skipFormula(formula, pos + 1));
					subexp = subexp.not();
					subexp2 = subexp2.not();
					subexp = subexp.and(subexp2);
					return subexp.not();
				default:
					List<String> dependencies = Arrays.asList("" + formula.charAt(pos));
					TransitionQuery var = new TransitionQuery();
					var.symbolMap.put(2, dependencies);
					List<TaskMDPTransition> succeed = new ArrayList<>();
					ObjectInstance o = new ObjectInstance(domain.getObjectClass(CLASSSPEC), CLASSSPEC);
					o.setValue(ATTSPEC, 1);
					succeed.add(new TaskMDPTransition(o, 1.0));
					var.transitions.put(new IntegerPair(2, 1), succeed);

					List<TaskMDPTransition> fail = new ArrayList<>();
					ObjectInstance fo = new ObjectInstance(domain.getObjectClass(CLASSSPEC), CLASSSPEC);
					fo.setValue(ATTSPEC, 0);
					fail.add(new TaskMDPTransition(fo, 1.0));
					var.transitions.put(new IntegerPair(2, 0), fail);
					return var;
			}
		}

		// Where is the next place to translate?
		public static int skipFormula(String formula, int pos) {
			switch (formula.charAt(pos)) {
				case 'F':
				case 'G':
					return skipFormula(formula, pos + 2);
				case '!':
					return skipFormula(formula, pos + 1);
				case '&':
				case '|':
					return skipFormula(formula, skipFormula(formula, pos + 1));
				case 'U':
					return skipFormula(formula, skipFormula(formula, pos + 2));
				default:
					return pos + 1;
			}
		}

		public void setTransitions(int taskstate, int actionlabel, List<TaskMDPTransition> transitionList) {
			transitions.put(new IntegerPair(taskstate, actionlabel), transitionList);
		}


		public void setDependency(int taskstate, List<String> dependencies) {
			symbolMap.put(taskstate, dependencies);
		}

		public List<String> symbolDependencies(int taskstate) {
			return symbolMap.get(taskstate);
		}

		public List<TaskMDPTransition> nextTaskStateTransitions(int taskstate, int actionlabel) {
			return transitions.get(new IntegerPair(taskstate, actionlabel));
		}

		public TransitionQuery not() {
			TransitionQuery negated = new TransitionQuery();
			negated.symbolMap = new HashMap<>(this.symbolMap);
			negated.transitions = new HashMap<>(this.transitions.size());

			for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e : this.transitions.entrySet()) {
				List<TaskMDPTransition> negatedTransitions = new ArrayList<TaskMDPTransition>(e.getValue().size());
				for (TaskMDPTransition trans : e.getValue()) {
					if (trans.taskObject.getIntValForAttribute(ATTSPEC) == 0) {
						ObjectInstance newtrans = trans.taskObject.copy();
						newtrans.setValue(ATTSPEC, 1);
						negatedTransitions.add(new TaskMDPTransition(newtrans, trans.p));
					} else if (trans.taskObject.getIntValForAttribute(ATTSPEC) == 1) {
						ObjectInstance newtrans = trans.taskObject.copy();
						newtrans.setValue(ATTSPEC, 0);
						negatedTransitions.add(new TaskMDPTransition(newtrans, trans.p));
					} else {
						negatedTransitions.add(new TaskMDPTransition(trans.taskObject.copy(), trans.p));
					}
				}
				negated.transitions.put(e.getKey(), negatedTransitions);
			}
			return negated;
		}

		public int getMaximumState() {
			int max = -1;

			for (Map.Entry<Integer, List<String>> e : this.symbolMap.entrySet()) {
					int id = e.getKey();

					max = Math.max(id, max);
			}
			return max;
		}

		// Let subexpression finish if it runs over time.
		public TransitionQuery eventuallyOLD(double discount) {
			TransitionQuery constructed = new TransitionQuery();
			constructed.symbolMap = new HashMap<>(this.symbolMap);
			constructed.transitions = new HashMap<>(this.transitions.size());

			int baseline = getMaximumState() + 1;

			// set symbol map for new states
			for (Map.Entry<Integer, List<String>> e : this.symbolMap.entrySet()) {
				int state = e.getKey();
				List<String> dependencies = e.getValue();
//				System.out.println(state+baseline + "+" + dependencies);
				if (state > 1) {
					constructed.symbolMap.put(state + baseline, dependencies);
				}
			}

			for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e : this.transitions.entrySet()) {
				List<TaskMDPTransition> constructedTransitions1 = new ArrayList<TaskMDPTransition>(e.getValue().size());
				List<TaskMDPTransition> constructedTransitions2 = new ArrayList<TaskMDPTransition>(e.getValue().size());

				// construct new destinations
				for (TaskMDPTransition trans : e.getValue()) {
					int olddest = trans.taskObject.getIntValForAttribute(ATTSPEC);
					if (olddest == 1) {
						ObjectInstance newtrans = trans.taskObject.copy();
						constructedTransitions1.add(new TaskMDPTransition(newtrans, trans.p));
						constructedTransitions2.add(new TaskMDPTransition(newtrans, trans.p));
					} else if (olddest == 0) {
						ObjectInstance newtransrestart = trans.taskObject.copy();
						ObjectInstance newtransfail = trans.taskObject.copy();
						ObjectInstance newtransexpire = trans.taskObject.copy();
						newtransrestart.setValue(ATTSPEC, 2);
						newtransexpire.setValue(ATTSPEC, 2 + baseline);
						constructedTransitions1.add(new TaskMDPTransition(newtransrestart, trans.p * discount));
						constructedTransitions1.add(new TaskMDPTransition(newtransexpire, trans.p * (1.0 - discount)));
						constructedTransitions2.add(new TaskMDPTransition(newtransfail, trans.p));
					} else {
						ObjectInstance newtrans1 = trans.taskObject.copy();
						ObjectInstance newtrans2 = trans.taskObject.copy();
						newtrans2.setValue(ATTSPEC, olddest + baseline);
						constructedTransitions1.add(new TaskMDPTransition(newtrans1, trans.p * discount));
						constructedTransitions1.add(new TaskMDPTransition(newtrans2, trans.p * (1.0 - discount)));
						constructedTransitions2.add(new TaskMDPTransition(newtrans2, trans.p));
					}
				}
				constructed.transitions.put(e.getKey(), constructedTransitions1);
				constructed.transitions.put(new IntegerPair(e.getKey().a + baseline, e.getKey().b), constructedTransitions2);
			}
			return constructed;
		}

        // formula must be true until op2 becomes true.
		public TransitionQuery until(double discount, TransitionQuery op2) {
			TransitionQuery constructed = new TransitionQuery();
			constructed.symbolMap = new HashMap<>(); // this.symbolMap);
			constructed.transitions = new HashMap<>(this.transitions.size());

			int baseline = op2.getMaximumState() + 1;

			// Try all pairs.
			for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e1 : this.transitions.entrySet()) {
				for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e2 : op2.transitions.entrySet()) {
					for (TaskMDPTransition trans1 : e1.getValue()) {
						for (TaskMDPTransition trans2 : e2.getValue()) {
							// get a pair of transitions from the old machines
							int from1 = e1.getKey().a;
							int from2 = e2.getKey().a;
							int actionlabel1 = e1.getKey().b;
							int actionlabel2 = e2.getKey().b;
							double prob1 = trans1.p;
							double prob2 = trans2.p;
							int to1 = trans1.taskObject.getIntValForAttribute(ATTSPEC);
							int to2 = trans2.taskObject.getIntValForAttribute(ATTSPEC);

							// configure the combined transition  for the new machine
							int from = until_combine(from1, from2, baseline);
							System.out.println("from ("+from1 + "," + from2+")="+from);
							int to = until_combine(to1, to2, baseline);
							System.out.println("to ("+to1 + "," + to2+")="+to);
							double prob = prob1 * prob2;
							int actionlabel = actionlabel1 * (1 << op2.symbolMap.get(from2).size()) + actionlabel2;

//							System.out.println(this.symbolMap + "||" + op2.symbolMap);
//							System.out.println(0/0);
							// Store things in the new machine
							List<TaskMDPTransition> currentlist = constructed.transitions.get(new IntegerPair(from, actionlabel));
							if (currentlist == null) currentlist = new ArrayList<>();

//							System.out.println(from + "->" + to + " " + actionlabel + " " + currentlist);
//							System.out.println(from1 + "->" + to1 + ", " + from2 + "->" + to2 + " => " + from + "->" + to);

							if (to > 1) { // not terminal
								ObjectInstance trans = trans1.taskObject.copy();
								trans.setValue(ATTSPEC, to);
								currentlist.add(new TaskMDPTransition(trans, prob*discount));
								ObjectInstance transfail = trans1.taskObject.copy();
								transfail.setValue(ATTSPEC, 0);
								currentlist.add(new TaskMDPTransition(transfail, prob*(1.0-discount)));
							} else { // terminal
								ObjectInstance transend = trans1.taskObject.copy();
								transend.setValue(ATTSPEC, to);
								currentlist.add(new TaskMDPTransition(transend, prob));
							}
							constructed.transitions.put(new IntegerPair(from, actionlabel), currentlist);

							List<String> dependencies1 = this.symbolDependencies(from1);
							List<String> dependencies2 = op2.symbolDependencies(from2);
							List<String> dependencies = new ArrayList<String>(dependencies1);
							dependencies.addAll(dependencies2);

							constructed.symbolMap.put(from, dependencies);
						}
					}
				}
			}
			return constructed;
		}

		// don't let subexpression finish if it runs over time.
		public TransitionQuery eventually(double discount) {
			TransitionQuery constructed = new TransitionQuery();
			constructed.symbolMap = new HashMap<>(this.symbolMap);
			constructed.transitions = new HashMap<>(this.transitions.size());

//			int baseline = getMaximumState() + 1;
//
//			// set symbol map for new states
//			for (Map.Entry<Integer, List<String>> e : this.symbolMap.entrySet()) {
//				int state = e.getKey();
//				List<String> dependencies = e.getValue();
////				System.out.println(state+baseline + "+" + dependencies);
//				if (state > 1) {
//					constructed.symbolMap.put(state + baseline, dependencies);
//				}
//			}

			// loop through transitions
			for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e : this.transitions.entrySet()) {
				List<TaskMDPTransition> constructedTransitions = new ArrayList<TaskMDPTransition>(e.getValue().size());

				// construct new destinations
				for (TaskMDPTransition trans : e.getValue()) {
					int olddest = trans.taskObject.getIntValForAttribute(ATTSPEC);
					if (olddest == 1) {
						ObjectInstance newtrans = trans.taskObject.copy();
						constructedTransitions.add(new TaskMDPTransition(newtrans, trans.p));
					} else if (olddest == 0) {
						ObjectInstance newtransrestart = trans.taskObject.copy();
						ObjectInstance newtransfail = trans.taskObject.copy();
						newtransrestart.setValue(ATTSPEC, 2);
						constructedTransitions.add(new TaskMDPTransition(newtransrestart, trans.p * discount));
						constructedTransitions.add(new TaskMDPTransition(newtransfail, trans.p * (1.0 - discount)));
					} else {
						ObjectInstance newtranscont = trans.taskObject.copy();
						ObjectInstance newtransexpire = trans.taskObject.copy();
						newtransexpire.setValue(ATTSPEC, 0); // modification
						constructedTransitions.add(new TaskMDPTransition(newtranscont, trans.p * discount));
						constructedTransitions.add(new TaskMDPTransition(newtransexpire, trans.p * (1.0 - discount)));
					}
				}
				constructed.transitions.put(e.getKey(), constructedTransitions);
			}
			return constructed;
		}

		public TransitionQuery and(TransitionQuery op2) {
			TransitionQuery constructed = new TransitionQuery();
			constructed.symbolMap = new HashMap<>(); // this.symbolMap);
			constructed.transitions = new HashMap<>(this.transitions.size());

			int baseline = op2.getMaximumState() + 1;

			// Try all pairs.
			for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e1 : this.transitions.entrySet()) {
				for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e2 : op2.transitions.entrySet()) {
					for (TaskMDPTransition trans1 : e1.getValue()) {
						for (TaskMDPTransition trans2 : e2.getValue()) {
							// get a pair of transitions from the old machines
							int from1 = e1.getKey().a;
							int from2 = e2.getKey().a;
							int actionlabel1 = e1.getKey().b;
							int actionlabel2 = e2.getKey().b;
							double prob1 = trans1.p;
							double prob2 = trans2.p;
							int to1 = trans1.taskObject.getIntValForAttribute(ATTSPEC);
							int to2 = trans2.taskObject.getIntValForAttribute(ATTSPEC);

							// configure the combined transition  for the new machine
							int from = and_combine(from1, from2, baseline);
							int to = and_combine(to1, to2, baseline);
							double prob = prob1 * prob2;
							int actionlabel = actionlabel1 * (1 << op2.symbolMap.get(from2).size()) + actionlabel2;

//							System.out.println(this.symbolMap + "||" + op2.symbolMap);
//							System.out.println(0/0);
							// Store things in the new machine
							List<TaskMDPTransition> currentlist = constructed.transitions.get(new IntegerPair(from, actionlabel));
							if (currentlist == null) currentlist = new ArrayList<>();

//							System.out.println(from + "->" + to + " " + actionlabel + " " + currentlist);
//							System.out.println(from1 + "->" + to1 + ", " + from2 + "->" + to2 + " => " + from + "->" + to);

							ObjectInstance trans = trans1.taskObject.copy();
							trans.setValue(ATTSPEC, to);
							currentlist.add(new TaskMDPTransition(trans, prob));
							constructed.transitions.put(new IntegerPair(from, actionlabel), currentlist);

							List<String> dependencies1 = this.symbolDependencies(from1);
							List<String> dependencies2 = op2.symbolDependencies(from2);
							List<String> dependencies = new ArrayList<String>(dependencies1);
							dependencies.addAll(dependencies2);

							constructed.symbolMap.put(from, dependencies);
						}
					}
				}
			}

			// from1 = 1, to1 = 1
			for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e2 : op2.transitions.entrySet()) {
				for (TaskMDPTransition trans2 : e2.getValue()) {
					// get a pair of transitions from the old machines
					int from1 = 1;
					int from2 = e2.getKey().a;
					int actionlabel = e2.getKey().b;
					double prob = trans2.p;
					int to1 = 1;
					int to2 = trans2.taskObject.getIntValForAttribute(ATTSPEC);

					// configure the combined transition  for the new machine
					int from = and_combine(from1, from2, baseline);
					int to = and_combine(to1, to2, baseline);

					// Store things in the new machine
					List<TaskMDPTransition> currentlist = constructed.transitions.get(new IntegerPair(from, actionlabel));
					if (currentlist == null) currentlist = new ArrayList<>();

//							System.out.println(from + "->" + to + " " + actionlabel + " " + currentlist);
//					System.out.println(from1 + "->" + to1 + ", " + from2 + "->" + to2 + " => " + from + "->" + to);

					ObjectInstance trans = trans2.taskObject.copy();
					trans.setValue(ATTSPEC, to);
					currentlist.add(new TaskMDPTransition(trans, prob));
					constructed.transitions.put(new IntegerPair(from, actionlabel), currentlist);

					List<String> dependencies2 = op2.symbolDependencies(from2);
					List<String> dependencies = new ArrayList<String>(dependencies2);

					constructed.symbolMap.put(from, dependencies);
				}
			}

			// from2 = 1, to2 = 1
			for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e1 : this.transitions.entrySet()) {
				for (TaskMDPTransition trans1 : e1.getValue()) {
					// get a pair of transitions from the old machines
					int from2 = 1;
					int from1 = e1.getKey().a;
					int actionlabel = e1.getKey().b;
					double prob = trans1.p;
					int to2 = 1;
					int to1 = trans1.taskObject.getIntValForAttribute(ATTSPEC);

					// configure the combined transition  for the new machine
					int from = and_combine(from1, from2, baseline);
					int to = and_combine(to1, to2, baseline);

					// Store things in the new machine
					List<TaskMDPTransition> currentlist = constructed.transitions.get(new IntegerPair(from, actionlabel));
					if (currentlist == null) currentlist = new ArrayList<>();

//							System.out.println(from + "->" + to + " " + actionlabel + " " + currentlist);
//					System.out.println(from1 + "->" + to1 + ", " + from2 + "->" + to2 + " => " + from + "->" + to);

					ObjectInstance trans = trans1.taskObject.copy();
					trans.setValue(ATTSPEC, to);
					currentlist.add(new TaskMDPTransition(trans, prob));
					constructed.transitions.put(new IntegerPair(from, actionlabel), currentlist);

					List<String> dependencies1 = this.symbolDependencies(from1);
					List<String> dependencies = new ArrayList<String>(dependencies1);

					constructed.symbolMap.put(from, dependencies);
				}
			}

			// Debugging
//			for (Map.Entry<Integer, List<String>> e : this.symbolMap.entrySet()) {
//				int state = e.getKey();
//				List<String> dependencies = e.getValue();
//				System.out.println(state + "/" + baseline + "+" + dependencies);
//				if (state > 1) {
//					constructed.symbolMap.put(state + baseline, dependencies);
//				}
//			}

			return constructed;
		}

		int and_combine(int node1, int node2, int baseline) {
			if ((node1 == 0) || (node2 == 0)) return 0;
			if ((node1 == 2) && (node2 == 2)) return 2;
			if ((node1 == 1) && (node2 == 1)) return 1;
			return node1*(baseline+2)+ node2;
		}

		// 1 needs to be true until 2 becomes true.
		// so we win if 2 is true. we lose if 1 is false.
		// restart if needed
		int until_combine(int node1, int node2, int baseline) {
			if (node2 == 1) return 1;
			if (node1 == 0) return 0;
			if (node1 == 1) node1 = 2; // great, keep making it true
			if (node2 == 0) node2 = 2; // fine, keep running since it hasn't become true yet.
			if ((node1 == 2) && (node2 == 2)) return 2;
			return node1*(baseline+2)+ node2;
		}
	}
}