#include <iostream>
#include <set>
#include <boost/atomic.hpp>
#include <boost/assign.hpp>
#include <boost/thread.hpp>
#include "cpphop.hpp"

using namespace cpphophtn;

typedef enum work_status_t{
	WORK_WAITING,
	WORK_WORKING,
	WORK_READY
}work_status_t;

// Contiene información de un trabajo a ser procesado
class work_t{
	public:
		unsigned long long id;
		void (*func)(work_t *w);
		void *param;
		std::vector<task> result;
		return_state_t result_status;
		work_status_t work_status;
		boost::mutex m;
		boost::condition_variable work_end;
		
		work_t ():work_status(WORK_WAITING) {}
		
		work_t (const work_t &w){
			id = w.id;
			func = w.func;
			param = w.param;
			result_status = w.result_status;
			work_status = w.work_status;
			result.clear();
			for(std::vector<task>::const_iterator it = w.result.begin(); it != w.result.end(); ++it){
				result.push_back(*it);
			}
		}
		
		void wait(){
			boost::mutex::scoped_lock lock(m);
			while(work_status != WORK_READY){
				work_end.wait(lock);
			}
		}
		
		inline work_t operator=(const work_t &w){
			id = w.id;
			func = w.func;
			param = w.param;
			result_status = w.result_status;
			work_status = w.work_status;
			result.clear();
			for(std::vector<task>::const_iterator it = w.result.begin(); it != w.result.end(); ++it){
				result.push_back(*it);
			}
			return *this;
		}
};

// Cola de trabajo, maneja todo el trabajo que se le imponga
class work_queue_t{
	public:
		work_queue_t():lastid(1), over(false) {}
		void start();
		unsigned long long add_work(work_t *w);
		work_status_t get_work_result(unsigned long long int id, bool wait, std::vector<task> &result);
		void end();
		void run();
		work_t *search(unsigned long long id);
		work_status_t erase(unsigned long long id);
		
	private:
		std::map<unsigned long long, work_t *> queue;
		boost::condition_variable work_paused;
		bool over;
		boost::mutex m;
		unsigned long long int lastid;
		boost::thread thr;
};

void work_queue_t::start(){
	thr = boost::thread(&work_queue_t::run, this);
}

void work_queue_t::run(){
	while(!over){
		boost::mutex::scoped_lock lock(m);
			while(queue.empty() && !over){
				work_paused.wait(lock);
			}
		lock.unlock();
		if(over)
			break;
		// work received
		unsigned long long id = 0;
		lock.lock();
			for(std::map<unsigned long long, work_t*>::iterator it = queue.begin(); it != queue.end();++it){
				if((*it).second->work_status == WORK_WAITING){
					id = (*it).first;
					break;
				}
			}
			if(id > 0){
				work_t *w = (*(queue.find(id))).second;
				lock.unlock();
					boost::mutex::scoped_lock wlock(w->m);
					w->work_status = WORK_WORKING;
					std::cout << "****************RUNNING FUNCTION***************" << std::endl << std::endl;
					if(w->func)
						w->func(w);
					std::cout << "****************FUNCTION ENDED***************" << std::endl << std::endl;
					w->work_status = WORK_READY;
					w->work_end.notify_one();
				lock.lock();
			}
		lock.unlock();
	}
}

void work_queue_t::end(){
	m.lock();
		over = true;
		queue.clear();
		work_paused.notify_one();
	m.unlock();
	thr.join();
}

unsigned long long work_queue_t::add_work(work_t *w){
	w->id = lastid;
	w->work_status = WORK_WAITING;
	boost::mutex::scoped_lock lock(m);
	queue.insert(std::pair<unsigned long long, work_t *>(lastid,w));
	lastid++;
	work_paused.notify_one();
	return w->id;
}

work_status_t work_queue_t::get_work_result(unsigned long long id, bool wait, std::vector<task> &result){
	work_t *w = search(id);
	if(!w){
		return WORK_WAITING;
	}
	if(wait){
		w->wait();
		return erase(id);
	}
	if(w->work_status != WORK_READY)
		return w->work_status;
	return erase(id);
}

work_t *work_queue_t::search(unsigned long long id){
	boost::mutex::scoped_lock lock(m);
	std::map<unsigned long long, work_t *>::iterator wit = queue.find(id);
	if(wit == queue.end())
		return NULL;
	work_t *w = (*wit).second;
	return w;
}

work_status_t work_queue_t::erase(unsigned long long id){
	boost::mutex::scoped_lock lock(m);
	std::map<unsigned long long, work_t *>::iterator it = queue.find(id);
	work_status_t status = ((*it).second)->work_status;
	queue.erase(it);
	return status;
}

using namespace cpphophtn;

// operators just like blocks_world_operator
bool pickup(state &s, std::map<std::string, boost::any>& parameters, state &final_state){
	final_state = s;
	std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(final_state.variables["pos"]);
	std::map<std::string, bool> clear = boost::any_cast< std::map<std::string, bool> >(final_state.variables["clear"]);
	std::string b = boost::any_cast<std::string>(parameters["b"]);
	std::string holding = boost::any_cast<std::string>(final_state.variables["holding"]);
	if(pos[b] == "table" && clear[b] == true && holding == ""){
		pos[b] = "hand";
		clear[b] = false;
		holding = b;
		final_state.variables["pos"] = pos;
		final_state.variables["clear"] = clear;
		final_state.variables["holding"] = holding;
		return true;
	}
	return false;
}

bool unstack(state &s, std::map<std::string, boost::any>& parameters, state &final_state){
	std::string b = boost::any_cast<std::string>(parameters["b"]);
	std::string c = boost::any_cast<std::string>(parameters["c"]);
	final_state = s;
	std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(final_state.variables["pos"]);
	std::map<std::string, bool> clear = boost::any_cast< std::map<std::string, bool> >(final_state.variables["clear"]);
	std::string holding = boost::any_cast<std::string>(final_state.variables["holding"]);
	if(pos[b] == c && c != "table" && clear[b] == true && holding == ""){
		pos[b] = "hand";
		clear[b] = false;
		holding = b;
		clear[c] = true;
		final_state.variables["pos"] = pos;
		final_state.variables["clear"] = clear;
		final_state.variables["holding"] = holding;
		return true;
	}
	return false;
}

bool putdown(state &s, std::map<std::string, boost::any>& parameters, state &final_state){
	std::string b = boost::any_cast<std::string>(parameters["b"]);
	final_state = s;
	std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(final_state.variables["pos"]);
	std::map<std::string, bool> clear = boost::any_cast< std::map<std::string, bool> >(final_state.variables["clear"]);
	std::string holding = boost::any_cast<std::string>(final_state.variables["holding"]);
	if(pos[b] == "hand"){
		pos[b] = "table";
		clear[b] = true;
		holding = "";
		final_state.variables["pos"] = pos;
		final_state.variables["clear"] = clear;
		final_state.variables["holding"] = holding;
		return true;
	}
	return false;
}

bool stack(state &s, std::map<std::string, boost::any>& parameters, state &final_state){
	std::string b = boost::any_cast<std::string>(parameters["b"]);
	std::string c = boost::any_cast<std::string>(parameters["c"]);
	final_state = s;
	std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(final_state.variables["pos"]);
	std::map<std::string, bool> clear = boost::any_cast< std::map<std::string, bool> >(final_state.variables["clear"]);
	std::string holding = boost::any_cast<std::string>(final_state.variables["holding"]);
	if(pos[b] == "hand" && clear[c] == true){
		pos[b] = c;
		clear[b] = true;
		holding = "";
		clear[c] = false;
		final_state.variables["pos"] = pos;
		final_state.variables["clear"] = clear;
		final_state.variables["holding"] = holding;
		return true;
	}
	return false;
}

// utility functions for methods just like blocks_world_methods

bool is_done(std::string& b1, state &s, state &goal){
	if(b1 == "table")
		return true;
	std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(goal.variables["pos"]);
	std::map<std::string, std::string> spos = boost::any_cast< std::map<std::string, std::string> >(s.variables["pos"]);
	std::map<std::string, std::string>::iterator it = pos.find(b1);
	if(it != pos.end() && pos[b1] != spos[b1]){
		return false;
	}
	if(spos[b1] == "table")
		return true;
	return is_done(spos[b1], s, goal);
}

std::string status(std::string& b1, state& s, state& goal){
	std::map<std::string, bool> clear = boost::any_cast< std::map<std::string, bool> >(s.variables["clear"]);
	std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(goal.variables["pos"]);
	std::map<std::string, std::string>::iterator it = pos.find(b1);
	if(is_done(b1, s, goal))
		return "done";
	else if(!clear[b1])
		return "inaccessible";
	else if(it == pos.end() || pos[b1] == "table")
		return "move-to-table";
	else if(is_done(pos[b1], s, goal) && clear[pos[b1]])
		return "move-to-block";
	else
		return "waiting";
}

void all_blocks(state& s, std::vector<std::string>& keys){
	std::map<std::string, bool> clear = boost::any_cast< std::map<std::string, bool> >(s.variables["clear"]);
	for(std::map<std::string, bool>::iterator it = clear.begin(); it != clear.end(); ++it)
		keys.push_back((*it).first);
}

// methods just like blocks_world_methods
bool moveb_m(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::vector<std::string> keys;
	all_blocks(init_state, keys);
	state goal = boost::any_cast<state>(parameters["goal"]);
	for(std::vector<std::string>::iterator it = keys.begin(); it != keys.end();it++){
		std::string &b1 = *it;
		std::string s = status(b1, init_state, goal);
		if(s == "move-to-table"){
			task t1;
			t1.name  = "move_one";
			t1.parameters["b1"] = boost::any(std::string(b1));
			t1.parameters["dest"] = boost::any(std::string("table"));
			tasks.push_back(t1);
			
			task t2;
			t2.name = "move_blocks";
			t2.parameters["goal"] = boost::any(goal);
			tasks.push_back(t2);
			return true;
		}else if(s == "move-to-block"){
			task t1;
			t1.name = "move_one";
			std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(goal.variables["pos"]);
			t1.parameters["b1"] = boost::any(std::string(b1));
			t1.parameters["dest"] = boost::any(std::string(pos[b1]));
			tasks.push_back(t1);
			
			task t2;
			t2.name = "move_blocks";
			t2.parameters["goal"] = boost::any(goal);
			tasks.push_back(t2);
			return true;
		}else{
			continue;
		}
	}
	std::string elem = "";
	for(std::vector<std::string>::iterator it = keys.begin(); it != keys.end();it++){
		std::string &b1 =  *it;
		if(status(b1, init_state, goal) == "waiting"){
			elem = b1;
			break;
		}
	}
	if(elem != ""){
		task t1;
		t1.name = "move_one";
		t1.parameters["b1"] = boost::any(std::string(elem));
		t1.parameters["dest"] = boost::any(std::string("table"));
		tasks.push_back(t1);
		
		task t2;
		t2.name = "move_blocks";
		t2.parameters["goal"] = boost::any(goal);
		tasks.push_back(t2);
		return true;
	}
	return true;
}

bool move1(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	task t1;
	std::string b1 = boost::any_cast<std::string>(parameters["b1"]);
	std::string dest = boost::any_cast<std::string>(parameters["dest"]);
	t1.name = "get";
	t1.parameters["b1"] = boost::any(std::string(b1));
	tasks.push_back(t1);
	
	task t2;
	t2.name = "put";
	t2.parameters["b1"] = boost::any(std::string(b1));
	t2.parameters["b2"] = boost::any(std::string(dest));
	tasks.push_back(t2);
	return true;
}

bool get_m(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::string b1 = boost::any_cast<std::string>(parameters["b1"]);
	std::map<std::string, bool> clear = boost::any_cast< std::map<std::string, bool> >(init_state.variables["clear"]);
	std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(init_state.variables["pos"]);
	if(clear[b1]){
		if(pos[b1] == "table"){
			task t;
			t.name = "pickup";
			t.parameters["b"] = boost::any(std::string(b1));
			tasks.push_back(t);
			return true;
		}else{
			task t;
			t.name = "unstack";
			t.parameters["b"] = boost::any(std::string(b1));
			t.parameters["c"] = boost::any(std::string(pos[b1]));
			tasks.push_back(t);
			return true;
		}
	}
	return false;
}

bool put_m(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::string b1 = boost::any_cast<std::string>(parameters["b1"]);
	std::string b2 = boost::any_cast<std::string>(parameters["b2"]);
	std::string holding = boost::any_cast<std::string>(init_state.variables["holding"]);
	if(holding == b1){
		if(b2 == "table"){
			task t;
			t.name = "putdown";
			t.parameters["b"] = boost::any(std::string(b1));
			tasks.push_back(t);
			return true;
		}else{
			task t;
			t.name = "stack";
			t.parameters["b"] = boost::any(std::string(b1));
			t.parameters["c"] = boost::any(std::string(b2));
			tasks.push_back(t);
			return true;
		}
	}
	return false;
}

bool get_by_unstack(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::string b1 = boost::any_cast<std::string>(parameters["b1"]);
	std::map< std::string, bool > clear = boost::any_cast< std::map< std::string, bool > >(init_state.variables["clear"]);
	if(clear[b1]){
		task t;
		t.name = "unstack_task";
		t.parameters["b1"] = boost::any(std::string(b1));
		tasks.push_back(t);
		return true;
	}
	return false;
}

bool get_by_pickup(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::string b1 = boost::any_cast<std::string>(parameters["b1"]);
	std::map< std::string, bool > clear = boost::any_cast< std::map< std::string, bool > >(init_state.variables["clear"]);
	if(clear[b1]){
		task t;
		t.name = "pickup_task";
		t.parameters["b1"] = boost::any(std::string(b1));
		tasks.push_back(t);
		return true;
	}
	return false;
}

bool pickup_m(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::string b1 = boost::any_cast<std::string>(parameters["b1"]);
	std::map< std::string, bool > clear = boost::any_cast< std::map< std::string, bool > >(init_state.variables["clear"]);
	if(clear[b1]){
		task t;
		t.name = "pickup";
		t.parameters["b"] = boost::any(std::string(b1));
		tasks.push_back(t);
		return true;
	}
	return false;
}

bool unstack_m(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::string b1 = boost::any_cast<std::string>(parameters["b1"]);
	std::map< std::string, bool > clear = boost::any_cast< std::map< std::string, bool > >(init_state.variables["clear"]);
	std::map<std::string, std::string> pos = boost::any_cast< std::map<std::string, std::string> >(init_state.variables["pos"]);
	if(clear[b1]){
		task t;
		t.name = "unstack";
		t.parameters["b"] = boost::any(std::string(b1));
		t.parameters["c"] = boost::any(std::string(pos[b1]));
		tasks.push_back(t);
		return true;
	}
	return false;
}

void declare_operators(cpphop &htn){
	htn_operator op;
	op.name = "pickup";
	op.action = pickup;
	htn.declare_operator("pickup", op);
	
	op.name = "unstack";
	op.action = unstack;
	htn.declare_operator("unstack", op);
	
	op.name = "putdown";
	op.action = putdown;
	htn.declare_operator("putdown", op);
	
	op.name = "stack";
	op.action = stack;
	htn.declare_operator("stack", op);
}

void declare_methods1(cpphop &htn){
	method m;
	std::vector<method> mtds;
	m.name = "moveb_m";
	m.method_exe = moveb_m;
	mtds.push_back(m);
	htn.declare_methods("move_blocks",  mtds);
	mtds.clear();
	
	m.name = "move1";
	m.method_exe = move1;
	mtds.push_back(m);
	htn.declare_methods("move_one", mtds);
	mtds.clear();
	
	m.name = "get_m";
	m.method_exe = get_m;
	mtds.push_back(m);
	htn.declare_methods("get", mtds);
	mtds.clear();
	
	m.name = "put";
	m.method_exe = put_m;
	mtds.push_back(m);
	htn.declare_methods("put", mtds);
	mtds.clear();

}

void declare_methods2(cpphop &htn){
	method m;
	std::vector<method> mtds;
	m.name = "moveb_m";
	m.method_exe = moveb_m;
	mtds.push_back(m);
	htn.declare_methods("move_blocks",  mtds);
	mtds.clear();
	
	m.name = "move1";
	m.method_exe = move1;
	mtds.push_back(m);
	htn.declare_methods("move_one", mtds);
	mtds.clear();
	
	m.name = "get_by_pickup";
	m.method_exe = get_by_pickup;
	mtds.push_back(m);
	m.name = "get_by_unstack";
	m.method_exe = get_by_unstack;
	mtds.push_back(m);
	htn.declare_methods("get", mtds);
	mtds.clear();
	
	m.name = "pickup_m";
	m.method_exe = pickup_m;
	mtds.push_back(m);
	htn.declare_methods("pickup_task", mtds);
	mtds.clear();
	
	m.name = "unstack_m";
	m.method_exe = unstack_m;
	mtds.push_back(m);
	htn.declare_methods("unstack_task", mtds);
	mtds.clear();
	
	m.name = "put";
	m.method_exe = put_m;
	mtds.push_back(m);
	htn.declare_methods("put", mtds);
	mtds.clear();
}

cpphop htn1;
void third_set_test1(work_t *w){
	std::cout << "*******************Thread working id: " << w->id << std::endl;
	
	state state2;
	std::cout << "****third set, another block****" << std::endl << std::endl;
	state2.name = "state2";
	std::map<std::string, std::string> pos = boost::assign::map_list_of ("g", "a") ("a", "c") ("c", "e") ("f", "d") ("d", "b") ("b", "h") ("e", "table") ("h", "table");
	state2.variables["pos"] = boost::any(pos);
	std::map<std::string, bool> clear = boost::assign::map_list_of ("g", true) ("a", false) ("c", false) ("e", false) ("f", true) ("d", false) ("b", false) ("h", false);
	state2.variables["clear"] = boost::any(clear);
	state2.variables["holding"] = boost::any(std::string(""));
	
	state goal2a;
	goal2a.name = "goal2a";
	pos = boost::assign::map_list_of("a", "b") ("b", "c") ("c", "d") ("e", "f") ("f", "g") ("g", "h") ("d", "table") ("h", "table");
	goal2a.variables["pos"] = boost::any(pos);
	clear = boost::assign::map_list_of("a", true) ("b", false) ("c", false) ("d", false) ("e", true) ("f", false) ("g", false) ("h", false);
	goal2a.variables["clear"] = boost::any(clear);
	goal2a.variables["holding"] = boost::any(std::string(""));
	
	std::vector<task> tasks;
	task t1;
	t1.name = "move_blocks";
	t1.parameters["goal"] = boost::any(goal2a);
	tasks.push_back(t1);
	
	std::cout << "- These should succeed" << std::endl;
	declare_operators(htn1);
	declare_methods1(htn1);
	std::vector<task> plan;
	std::cout << "###################### RUNNING PLAN 1 #####################"<< std::endl << std::endl;
	htn1.plan(state2, tasks, plan, 3, 0);
	std::cout << "###################### PLAN 1 ENDED #####################"<< std::endl << std::endl;
}

cpphop htn2;
void third_set_test2(work_t *w){
	std::cout << "*******************Thread working id: " << w->id << std::endl;
	
	state state2;
	std::cout << "****third set, another block****" << std::endl << std::endl;
	state2.name = "state2";
	std::map<std::string, std::string> pos = boost::assign::map_list_of("a", "c") ("b", "d") ("c", "table") ("d", "table");
	state2.variables["pos"] = boost::any(pos);
	std::map<std::string, bool> clear = boost::assign::map_list_of("a", true) ("c", false) ("b", true) ("d", false);
	state2.variables["clear"] = boost::any(clear);
	state2.variables["holding"] = boost::any(std::string(""));
	
	state goal2b;
	goal2b.name = "goal2b";
	pos = boost::assign::map_list_of("b", "c") ("a", "d") ;
	goal2b.variables["pos"] = boost::any(pos);
	
	std::vector<task> tasks;
	task t2;
	t2.name = "move_blocks";
	t2.parameters["goal"] = boost::any(goal2b);
	tasks.push_back(t2);
	
	htn2.clear();
	declare_operators(htn2);
	declare_methods1(htn2);
	std::vector<task> plan;
	std::cout << "###################### RUNNING PLAN 2 #####################"<< std::endl << std::endl;
	htn2.plan(state2, tasks, plan, 3, 0);
	std::cout << "###################### PLAN 2 ENDED #####################"<< std::endl << std::endl;
}

void resume_set_test1(work_t *w){
	
	state state;
	std::vector<task> tasks;
	std::vector<task> result;
	int verbose;
	suseconds_t miliseconds;
	
	std::cout << "###################### RESUMING PLAN 1 #####################"<< std::endl << std::endl;
	htn1.plan(state, tasks, result, verbose, miliseconds);
	std::cout << "###################### PLAN 1 ENDED #####################"<< std::endl << std::endl;

}

int main(){
	
	work_t w;
	w.func = third_set_test1;
	
	work_queue_t q;
	q.start();
	unsigned long long id1 = q.add_work(&w);
	work_t w2;
	w2.func = third_set_test2;
	unsigned long long id2 = q.add_work(&w2);
	
	std::vector<task> r;
	bool iterate = true;
	while(iterate){
		work_status_t ws = q.get_work_result(id1, false, r);
		switch(ws){
			case WORK_READY:
				std::cout << "PLAN 1 PAUSED!!!" << std::endl;
				iterate = false;
				break;
			case WORK_WORKING:
				std::cout << "PAUSING PLAN 1!!!!" << std::endl;
				if(htn1.pause_plan()){
					work_status_t ws = q.get_work_result(id1, true, r);
					iterate = false;
				}
				break;
		}
	}
	iterate = true;
	
	while(iterate){
		work_status_t ws = q.get_work_result(id2, true, r);
		switch(ws){
			case WORK_READY:
				iterate = false;
				break;
		}
	}
	
	work_t w3;
	w3.func = resume_set_test1;
	unsigned long long id3 = q.add_work(&w3);
	iterate = true;
	
	while(iterate){
		work_status_t ws = q.get_work_result(id3, true, r);
		switch(ws){
			case WORK_READY:
				iterate = false;
				break;
		}
	}
	q.end();
	return 0;
}

