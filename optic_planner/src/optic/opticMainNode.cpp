/************************************************************************
 * Copyright 2012; Planning, Agents and Intelligent Systems Group,
 * Department of Informatics,
 * King's College, London, UK
 * http://www.inf.kcl.ac.uk/staff/andrew/planning/
 *
 * Amanda Coles, Andrew Coles - OPTIC
 * Amanda Coles, Andrew Coles, Maria Fox, Derek Long - POPF
 * Stephen Cresswell - PDDL Parser
 *
 * This file is part of OPTIC.
 *
 * OPTIC is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * OPTIC is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OPTIC.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "ptree.h"
#include <assert.h>
#include <FlexLexer.h>
#include "instantiation.h"
#include "SimpleEval.h"
#include "DebugWriteController.h"
#include "typecheck.h"
#include "TIM.h"
#include "FuncAnalysis.h"

//#include "graphconstruct.h"
#include "RPGBuilder.h"
#include "FFSolver.h"
#include "globals.h"
#ifdef TOTALORDERSTATES
#include "colintotalordertransformer.h"
#else
#include "totalordertransformer.h"
#include "partialordertransformer.h"
#endif
#include "lpscheduler.h"
#include "numericanalysis.h"
#include "PreferenceHandler.h"

#ifdef STOCHASTICDURATIONS
#include "StochasticDurations.h"
#endif

#include <sys/times.h>
#include <unistd.h>

#include <sstream>
#include "temporalanalysis.h"

#include "rclcpp/rclcpp.hpp"

using std::ifstream;
using std::cerr;
using std::endl;
using std::ostringstream;
using std::istringstream;

using namespace TIM;
using namespace Inst;
using namespace VAL;
using namespace Planner;

namespace VAL
{
bool ContinueAnyway;
bool ErrorReport;
bool InvariantWarnings;
bool LaTeX;
bool makespanDefault;
};

class OpticNode : public rclcpp::Node
{
public:
    OpticNode()
    : rclcpp::Node("optic_node")
    {
        declare_parameter<std::int>("timeout", int(10));
        declare_parameter<std::string>("domain_path", std::string(""));
        declare_parameter<std::string>("problem_path", std::string(""));
        declare_parameter<std::string>("output_path", std::string(""));
    }
    void initialize()
    {
        get_parameter("timeout", timeout_);
        get_parameter("domain_path", domain_path_);
        get_parameter("problem_path", problem_path_);
        get_parameter("output_path", output_path_);
        
    }
    void solve()
    {
        FAverbose = false;

        int argcount = 1;

        FF::steepestDescent = false;
        FF::incrementalExpansion = false;
        FF::invariantRPG = false;
        FF::timeWAStar = false;
        LPScheduler::hybridBFLP = false;

        bool benchmark = false;
        bool readInAPlan = false;
        bool postHocTotalOrder = false;
        bool debugPreprocessing = false;
        bool postHocScheduleToMetric = false;
        if(timeout_ > 0)
        {
            Globals::timeLimit = timeout_;
        }
        /*
        #ifdef STOCHASTICDURATIONS
        const int expectFromHere = 3;
        #else
        const int expectFromHere = 2;
        #endif

        performModernTIMAnalysis(domain_path_, problem_path_);

        cout << std::setprecision(3) << std::fixed;

        #ifdef STOCHASTICDURATIONS
        setDurationManager(durationManagerString);
        #endif

        #ifdef TOTALORDERSTATES
        MinimalState::setTransformer(new TotalOrderTransformer());
        #else
        if (Globals::totalOrder) {
            MinimalState::setTransformer(new TotalOrderTransformer());
        } else {
            MinimalState::setTransformer(new PartialOrderTransformer());
        }
        #endif

        std::string plan_name = "plan_to_fix";
        #ifdef ENABLE_DEBUGGING_HOOKS
        if (debugPreprocessing) {
            Globals::planFilename = plan_name;
        }
        #endif

        #ifdef POPF3ANALYSIS
        const bool realOpt = Globals::optimiseSolutionQuality;
        Globals::optimiseSolutionQuality = (Globals::optimiseSolutionQuality || postHocScheduleToMetric);
        #endif

        RPGBuilder::initialise();

        #ifdef POPF3ANALYSIS
        Globals::optimiseSolutionQuality = realOpt;
        #endif

        #ifdef STOCHASTICDURATIONS
        initialiseDistributions();
        setSolutionDeadlineTimeToLatestGoal();
        #endif

        if (Globals::optimiseSolutionQuality && Globals::givenSolutionQualityDefined) {
            if (RPGBuilder::getMetric()) {
                cout << "Forcing the use of the given solution quality of " << Globals::givenSolutionQuality << endl;
                if (RPGBuilder::getMetric()->minimise) {
                    Globals::bestSolutionQuality = (Globals::givenSolutionQuality == 0.0 ? 0.0 : -Globals::givenSolutionQuality);
                } else {
                    Globals::bestSolutionQuality = Globals::givenSolutionQuality;
                }

                RPGBuilder::getHeuristic()->metricHasChanged();
            }
        }


        bool reachesGoals;

        Solution planAndConstraints;

        list<FFEvent> * & spSoln = planAndConstraints.plan;
        if (readInAPlan) {
            spSoln = readPlan(plan_name);
            reachesGoals = true;
    #ifndef NDEBUG
            spSoln = FF::doBenchmark(reachesGoals, spSoln, false);
    #endif
        } else {
            planAndConstraints = FF::search(reachesGoals);
        }
        cout << "HEREEEEE!!!" << endl;
        // cout << FFEvent::printPlan(*planAndConstraints.plan);
        cout << "HEREEEEE!!!" << endl;

        if (spSoln) {

            for (int pass = 0; pass < 2; ++pass) {
                if (pass) {
                    if (!postHocScheduleToMetric) break;
                    #ifndef TOTALORDERSTATES
                    if (!spSoln->empty()) {
                        if (Globals::totalOrder && !postHocTotalOrder) {
                            MinimalState::setTransformer(new PartialOrderTransformer());
                            Globals::totalOrder = false;
                            FF::tsChecking = false;
                        }
                        assert(planAndConstraints.constraints);
                        spSoln = FF::reprocessPlan(spSoln, planAndConstraints.constraints);
                        planAndConstraints.constraints = 0;
                    }
                    cout << ";;;; Post-hoc optimised solution\n";
                    #endif
                } else {
                    cout << ";;;; Solution Found\n";
                    cout << "; States evaluated: " << RPGHeuristic::statesEvaluated << endl;
                    cout << "; Cost: " << planAndConstraints.quality << endl;
                }
                cout << "HEREEEEE!!!" << endl;
                std::ofstream outFile(output_path_);
                if (outFile.is_open()) {
                    FFEvent::printPlanStream(*spSoln, outFile);
                    outFile.close();
                } else {
                    std::cerr << "Errore nell'aprire il file di output!" << std::endl;
                }               

                cout << "HEREEEEEE!!" << endl;
            }

            if (benchmark) {
                FF::doBenchmark(reachesGoals, spSoln);
            }

            return 0;
        } else {
            cout << ";; Problem unsolvable!\n";
            tms refReturn;
            times(&refReturn);
            double secs = ((double)refReturn.tms_utime + (double)refReturn.tms_stime) / ((double) sysconf(_SC_CLK_TCK));

            int twodp = (int)(secs * 100.0);
            int wholesecs = twodp / 100;
            int centisecs = twodp % 100;

            cout << "; Time " << wholesecs << ".";
            if (centisecs < 10) cout << "0";
            cout << centisecs << "\n";
            return 1;
        }
        */

    }

private:
    std::string domain_path_;
    std::string problem_path_;
    std::string output_path_;
    int timeout_;

}

list<FFEvent> * readPlan(char* filename);



extern int yyparse();
extern int yydebug;

void split(const int & insAt, list<FFEvent>::iterator insStart, const list<FFEvent>::iterator & insItr, const list<FFEvent>::iterator & insEnd)
{

    {
        for (; insStart != insItr; ++insStart) {
            int & currPWS = insStart->pairWithStep;
            if (currPWS != -1) {
                if (currPWS >= insAt) {
                    ++currPWS;
                }
            }
        }
    }
    {
        list<FFEvent>::iterator insPost = insItr;
        for (; insPost != insEnd; ++insPost) {
            int & currPWS = insPost->pairWithStep;
            if (currPWS != -1) {
                if (insPost->time_spec == Planner::E_AT_START) {
                    ++currPWS;
                } else if (insPost->time_spec == Planner::E_AT_END) {
                    if (currPWS >= insAt) {
                        ++currPWS;
                    }
                }
            }
        }
    }
}

namespace VAL
{
extern yyFlexLexer* yfl;
};

list<FFEvent> * readPlan(char* filename)
{
    static const bool debug = true;

    ifstream * const current_in_stream = new ifstream(filename);
    if (!current_in_stream->good()) {
        cout << "Exiting: could not open plan file " << filename << "\n";
        exit(1);
    }

    VAL::yfl = new yyFlexLexer(current_in_stream, &cout);
    yyparse();

    VAL::plan * const the_plan = dynamic_cast<VAL::plan*>(top_thing);

    delete VAL::yfl;
    delete current_in_stream;



    if (!the_plan) {
        cout << "Exiting: failed to load plan " << filename << "\n";
        exit(1);
    };

    if (!theTC->typecheckPlan(the_plan)) {
        cout << "Exiting: error when type-checking plan " << filename << "\n";
        exit(1);
    }

    list<FFEvent> * const toReturn = new list<FFEvent>();

    pc_list<plan_step*>::const_iterator planItr = the_plan->begin();
    const pc_list<plan_step*>::const_iterator planEnd = the_plan->end();

    for (int idebug = 0, i = 0; planItr != planEnd; ++planItr, ++i, ++idebug) {
        plan_step* const currStep = *planItr;

        instantiatedOp * const currOp = instantiatedOp::findInstOp(currStep->op_sym, currStep->params->begin(), currStep->params->end());
        if (!currOp) {
            const instantiatedOp * const debugOp = instantiatedOp::getInstOp(currStep->op_sym, currStep->params->begin(), currStep->params->end());
            cout << "Exiting: step " << idebug << " in the input plan uses the action " << *(debugOp) << ", which the instantiation code in the planner does not recognise.\n";
            exit(1);
        }
        const int ID = currOp->getID();

        if (RPGBuilder::getRPGDEs(ID).empty()) {// non-durative action
            FFEvent toInsert(currOp, 0.001, 0.001);
            const double ts = currStep->start_time;
            if (debug) cout << "; input " << ts << ": " << *currOp << " (id=" << ID << "), non-temporal";
            toInsert.lpTimestamp = ts;
            toInsert.lpMinTimestamp = ts;
            int insAt = 0;
            list<FFEvent>::iterator insItr = toReturn->begin();
            const list<FFEvent>::iterator insEnd = toReturn->end();
            for (; insItr != insEnd; ++insItr, ++insAt) {
                if (ts < insItr->lpTimestamp) {
                    split(insAt, toReturn->begin(), insItr, insEnd);
                    toReturn->insert(insItr, toInsert);
                    break;
                }
            }
            if (insItr == insEnd) {
                toReturn->push_back(toInsert);
            }
            if (debug) cout << " putting at step " << insAt << "\n";
        } else {
            int startIdx = -1;
            list<FFEvent>::iterator startIsAt = toReturn->end();
            const double actDur = currStep->duration;
            for (int pass = 0; pass < 2; ++pass) {
                if (pass) assert(startIdx >= 0);
                const double ts = (pass ? currStep->start_time + actDur : currStep->start_time);
                if (debug) {
                    cout << "; input " << ts << ": " << *currOp;
                    if (pass) {
                        cout << " end";
                    } else {
                        cout << " start";
                    }
                    cout << " (id=" << ID << ")";
                }
                FFEvent toInsert = (pass ? FFEvent(currOp, startIdx, actDur, actDur) : FFEvent(currOp, actDur, actDur));
                toInsert.lpTimestamp = ts;
                toInsert.lpMinTimestamp = ts;

                list<FFEvent>::iterator insItr = toReturn->begin();
                const list<FFEvent>::iterator insEnd = toReturn->end();
                int insAt = 0;
                for (; insItr != insEnd; ++insItr, ++insAt) {
                    if (ts < insItr->lpTimestamp) {
                        split(insAt, toReturn->begin(), insItr, insEnd);
                        const list<FFEvent>::iterator dest = toReturn->insert(insItr, toInsert);
                        if (pass) {
                            startIsAt->pairWithStep = insAt;
                            if (debug) cout << " putting at step " << insAt << ", pairing with " << startIdx << "\n";
                        } else {
                            startIsAt = dest;
                            startIdx = insAt;
                            if (debug) cout << " putting at step " << insAt << "\n";
                        }
                        break;
                    }
                }
                if (insItr == insEnd) {
                    toReturn->push_back(toInsert);
                    if (pass) {
                        startIsAt->pairWithStep = insAt;
                        if (debug) cout << " putting at step " << insAt << ", pairing with " << startIdx << "\n";
                    } else {
                        startIsAt = toReturn->end();
                        --startIsAt;
                        startIdx = insAt;
                        if (debug) cout << " putting at step " << insAt << "\n";
                    }
                }

            }
        }
    }

    const vector<RPGBuilder::FakeTILAction*> & tils = RPGBuilder::getNonAbstractedTILVec();
    const int tilCount = tils.size();

    for (int t = 0; t < tilCount; ++t) {
        FFEvent toInsert(t);
        const double tilTS = tils[t]->duration;
        toInsert.lpMaxTimestamp = tilTS;
        toInsert.lpMinTimestamp = tilTS;
        toInsert.lpTimestamp = tilTS;

        if (debug) {
            cout << "TIL " << toInsert.divisionID << " goes at " << tilTS << endl;
        }

        list<FFEvent>::iterator insItr = toReturn->begin();
        const list<FFEvent>::iterator insEnd = toReturn->end();
        for (int insAt = 0; insItr != insEnd; ++insItr, ++insAt) {
            if (tilTS < insItr->lpTimestamp) {
                split(insAt, toReturn->begin(), insItr, insEnd);
                toReturn->insert(insItr, toInsert);
                break;
            }
        }
        if (insItr == insEnd) {
            toReturn->push_back(toInsert);
        }
    }

    if (debug) {
        list<FFEvent>::iterator insItr = toReturn->begin();
        const list<FFEvent>::iterator insEnd = toReturn->end();

        for (int i = 0; insItr != insEnd; ++insItr, ++i) {
            cout << i << ": ";
            if (insItr->action) {
                cout << *(insItr->action);
                if (insItr->time_spec == Planner::E_AT_START) {
                    cout << " start\n";
                } else {
                    cout << " end\n";
                }
            } else {
                cout << "TIL " << insItr->divisionID << endl;
            }
        }
    }

    return toReturn;
};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpticNode>();
    node->initialize();
    node->solve();
    rclcpp::shutdown();
    return 0;
}