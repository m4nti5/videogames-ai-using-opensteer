#include <iostream>
#include "OpenSteer/cpphop.hpp"

using namespace cpphophtn;
/*
    Try to find a plan that accomplishes tasks in state. 
    If successful, return true. Otherwise return False.
*/
bool cpphop::plan(state& state, std::vector<task> &tasks, std::vector<task>& result, int verbose){
    if(verbose > 0){
    	std::cout << "** cpphop, verbose= " << verbose <<  std::endl << "** state = " << state << std::endl << "** tasks = " << tasks;
    }
    bool runned = this->seek_plan(state, tasks, 0, result, verbose);
    if(verbose > 0){
    	runned ? std::cout << "** result: " << result << std::endl : std::cout << "** result: IMPOSSIBRU!" << std::endl << std::endl;
    }
    return runned;
}

/*
    Workhorse for cpphop. state and tasks are as in cpphop.
    - plan is the current partial plan.
    - depth is the recursion depth, for use in debugging
    - verbose is whether to print debugging messages
*/
bool cpphop::seek_plan(state& current_state, std::vector<task>& tasks, int depth, std::vector<task>& plan, int verbose){
    if(verbose > 1){
    	std::cout << "depth " << depth << " tasks " << tasks;
    }
    if(tasks.size() == 0){
        if(verbose > 2){
        	std::cout << "depth " << depth << " plan " << plan << std::endl;
        }
        return true;
    }
    
    task task1 = tasks[0];
    htn_operator op;
    bool found_op = get_operator(task1.name, op);
    if(found_op){
    	if(verbose > 2){
    		std::cout << "depth " << depth << " action " << task1 << " operator: " << op.name << std::endl;
    	}
    	state newstate;
    	if(op.action(current_state, task1.parameters, newstate)){
    		if(verbose > 2){	
		    	std::cout << "depth " << depth << " newstate " << newstate << std::endl;
	    	}
    		std::vector<task> newtasks (tasks.begin() + 1, tasks.end());
    		task t;
    		t.name = op.name;
    		t.parameters = task1.parameters;
    		plan.push_back(t);
    		bool solution = seek_plan(newstate, newtasks, depth + 1, plan, verbose);
    		if(solution)
    			return solution;
    	}
    }
    
    std::vector<method> relevant;
    if(get_methods(task1.name, relevant)){
		if(verbose > 2){	
	    	std::cout << "depth " << depth << " method instance " << task1 << std::endl;
    	}
    	std::vector<method>::iterator it;
    	for(it = relevant.begin(); it != relevant.end(); ++it){
    		method m = (*it);
    		std::vector<task> subtasks;
    		if(m.method_exe(current_state, task1.parameters, subtasks)){
				if(verbose > 2){		
					std::cout << "depth " << depth << " new tasks " << subtasks;
				}
				for(std::vector<task>::iterator i = tasks.begin() + 1; i != tasks.end(); ++i){
					subtasks.push_back(*i);
				}
    			bool solution = seek_plan(current_state, subtasks, depth + 1, plan, verbose);
    			if(solution)
    				return solution;
    		}
    	}
    }
	if(verbose > 2){	
    	std::cout << "depth " << depth << " returns failure" << std::endl;
	}
	return false;
}

