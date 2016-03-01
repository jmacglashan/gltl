package gltl.demo;

import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteMaskHashingFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldRewardFunction;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.singleagent.GroundedAction;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by mlittman on 9/1/15.
 */
public class RnScan {
    public static void main(String[] args) {
        double slipProb = 0.2;
        double stepCost = -.04;
        double stayProb;

        BufferedWriter out = null;

        try {
            out = new BufferedWriter(new FileWriter("all.pol"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        for (slipProb = 0.0; slipProb <= 1.0; slipProb += .01) {
            for (stepCost = -1.65; stepCost <= 0.02; stepCost += .01) {
                GridWorldDomain gwd = new GridWorldDomain(4, 3);
                gwd.setObstacleInCell(1, 1);
                stayProb = 1.0 - slipProb;
                double thisSlip = slipProb * .5;
                double[][] transitionDynamics = new double[][]
                        {
                                // North   South     East      West
                                {stayProb, 0., thisSlip, thisSlip}, //selecting north
                                {0., stayProb, thisSlip, thisSlip}, //selecting south
                                {thisSlip, thisSlip, stayProb, 0.},       //selecting east
                                {thisSlip, thisSlip, 0., stayProb}  // selecting west
                        };
                gwd.setTransitionDynamics(transitionDynamics);
                Domain domain = gwd.generateDomain();

                GridWorldRewardFunction rf = new GridWorldRewardFunction(4, 3, stepCost);
/* start
                rf.setReward(3, 2, 1. + stepCost);
                rf.setReward(3, 1, -1. + stepCost);

                GridWorldTerminalFunction tf = new GridWorldTerminalFunction(3, 2);
                tf.markAsTerminalPosition(3, 1);
stop */
/* start */
                rf.setReward(3, 2, 1. + stepCost);
                rf.setReward(3, 1, -1. + stepCost);
                rf.setReward(3, 0, -1. + stepCost);
                rf.setReward(1, 2, -1. + stepCost);

                GridWorldTerminalFunction tf = new GridWorldTerminalFunction(3, 2);
/* stop */


                ValueIteration vi = new ValueIteration(domain, rf, tf, 1.0, new DiscreteMaskHashingFactory(), 0.0001, 100000);
                vi.toggleDebugPrinting(false);

                State s = GridWorldDomain.getOneAgentNoLocationState(domain, 0, 0);
                vi.planFromState(s);
                Policy p = new GreedyQPolicy(vi);

                String policy = "";
                for (int xp = 0; xp < 4; xp++) {
                    for (int yp = 0; yp < 3; yp++) {
                        if ((xp == 1) && (yp == 1)) continue;
                        // if ((xp == 3) && (yp == 1)) continue;
                        // if ((xp == 3) && (yp == 2)) continue;

                        s = GridWorldDomain.getOneAgentNoLocationState(domain, xp, yp);
                        GroundedAction ga = (GroundedAction) p.getAction(s);
                        policy = policy + ga.toString();
                    }
                }

                try {
                    out.write(policy + " " + stepCost + " " + slipProb + "\n");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}

