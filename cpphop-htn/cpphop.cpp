#include <iostream>
#include "cpphop.hpp"

using namespace cpphophtn;

void cpphop::load_pause_info(state& state, std::vector<task>& tasks, int& depth, std::vector<task>& result){
	paused_info.saved = false;
	state = paused_info.saved_state;
	tasks.clear();
	for(std::vector<task>::iterator it = paused_info.tasks.begin(); it != paused_info.tasks.end(); ++it)
		tasks.push_back(*it);
	depth = paused_info.depth;
	result.clear();
	for(std::vector<task>::iterator it = paused_info.result.begin(); it != paused_info.result.end(); ++it)
		result.push_back(*it);
	mutex.lock();
		paused = false;
		pause = false;
	mutex.unlock();
}

void cpphop::save_pause_info(state& state, std::vector<task>& tasks, int& depth, std::vector<task>& result){
	paused_info.saved = true;
	paused_info.saved_state = state;
	paused_info.tasks.clear();
	for(std::vector<task>::iterator it = tasks.begin(); it != tasks.end(); ++it)
		paused_info.tasks.push_back(*it);
	paused_info.depth = depth;
	paused_info.result.clear();
	for(std::vector<task>::iterator it = result.begin(); it != result.end(); ++it)
		paused_info.result.push_back(*it);
	if(pause){
		boost::mutex::scoped_lock lock(mutex);
		paused = true;
		plan_paused.notify_one();
	}
}

/*
    Try to find a plan that accomplishes tasks in state. 
    If successful, return true. Otherwise return False.
*/
return_state_t cpphop::plan(state& state, std::vector<task> &tasks, std::vector<task>& result, int verbose, suseconds_t miliseconds){
	mutex.lock();
		running = true;
	mutex.unlock();
    if(verbose > 0){
    	std::cout << "** cpphop, verbose= " << verbose <<  std::endl << "** state = " << state << std::endl << "** tasks = " << tasks;
    }
    int depth = 0;
    if(paused_info.saved){
    	if(verbose > 0)
    		std::cout << "** cpphop, loading saved info" << std::endl;
    	
    	load_pause_info(state, tasks, depth, result);
    }
    return_state_t runned = seek_plan(state, tasks, depth, result, verbose, miliseconds, 0);
    if(verbose > 0){
    	switch(runned){
    		case STATE_TRUE:
    			std::cout << "** result: " << result << std::endl;
    			break;
    		case STATE_FALSE:
    			std::cout << "** result: IMPOSSIBRU!" << std::endl << std::endl;
    			break;
    		case STATE_PAUSED:
    			std::cout << "** PAUSED!" << std::endl << std::endl;
    			break;
    	}
    }
    mutex.lock();
		if(!paused){
			running = false;
		}
	mutex.unlock();
    return runned;
}

/*
    Workhorse for cpphop. state and tasks are as in cpphop.
    - plan is the current partial plan.
    - depth is the recursion depth, for use in debugging
    - verbose is whether to print debugging messages
*/
return_state_t cpphop::seek_plan(state& current_state, std::vector<task>& tasks, int depth, std::vector<task>& plan, int verbose, suseconds_t miliseconds, suseconds_t time_elapsed){
	struct timeval tp;
	struct timezone tz;
	gettimeofday(&tp, &tz);
	// time measurement in miliseconds
	suseconds_t ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    if(verbose > 1){
    	std::cout << "depth " << depth << " tasks " << tasks;
    }
    if(tasks.size() == 0){
        if(verbose > 2){
        	std::cout << "depth " << depth << " plan " << plan << std::endl;
        }
        return STATE_TRUE;
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
			gettimeofday(&tp, &tz);
			suseconds_t runtime = (tp.tv_sec * 1000 + tp.tv_usec / 1000) - ms;
			return_state_t solution;
			bool p;
			mutex.lock();
				p = pause;
			mutex.unlock();
    		if((miliseconds == 0 || runtime + time_elapsed < miliseconds) && !p){
	    		solution = seek_plan(newstate, newtasks, depth + 1, plan, verbose, miliseconds, time_elapsed + runtime);
	    	}else{
	    		if(verbose > 0){
					if(!p)
						std::cout << "Time expired, saving info... " << runtime + time_elapsed << std::endl;
					else
						std::cout << "Paused by other thread... " << runtime + time_elapsed << std::endl;
		    	}
	    		int d = depth + 1;
	    		save_pause_info(newstate, newtasks, d, plan);
	    		solution = STATE_PAUSED;
	    	}
    		if(solution == STATE_TRUE || solution == STATE_PAUSED)
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
				gettimeofday(&tp, &tz);
				suseconds_t runtime = (tp.tv_sec * 1000 + tp.tv_usec / 1000) - ms;
				return_state_t solution;
				bool p;
				mutex.lock();
					p = pause;
				mutex.unlock();
				if((miliseconds == 0 || runtime + time_elapsed < miliseconds) && !p){
					solution = seek_plan(current_state, subtasks, depth + 1, plan, verbose, miliseconds, time_elapsed + runtime);
				}else{
					if(verbose > 0)	{
						if(!p)
				    		std::cout << "Time expired, saving info... " << runtime + time_elapsed << std::endl;
				    	else
							std::cout << "Paused by other thread... " << runtime + time_elapsed << std::endl;
		    		}
		    		int d = depth + 1;
					save_pause_info(current_state, subtasks, d, plan);
					solution = STATE_PAUSED;
				}
				if(solution == STATE_TRUE || solution == STATE_PAUSED)
						return solution;
				}
    	}
    }
	if(verbose > 2){
    	std::cout << "depth " << depth << " returns failure" << std::endl;
	}
	return STATE_FALSE;
}

bool cpphop::pause_plan(){
	if(!running)
		return false;
	boost::mutex::scoped_lock lock(mutex);
	pause = true;
	while(!paused){
		plan_paused.wait(lock);
	}
	return true;
}

bool compare_plans(const std::vector<task> & p1, const std::vector<task> & p2){
	if(p1.size() != p2.size())
		return false;
	std::vector<task>::const_iterator p2_it = p2.begin();
	for(std::vector<task>::const_iterator it = p1.begin(); it != p1.end(); ++it,p2_it++){
		if((*it).name != (*p2_it).name)
			return false;
	}
	return true;
}


