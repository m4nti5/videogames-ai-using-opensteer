#include <iostream>
#include <set>
#include <boost/assign.hpp>
#include <boost/thread.hpp>
#include "cpphop.hpp"

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

// first_set_test very simple, small tasks and operators

state state1;
void first_set_test(){
	std::cout << "****first_set_test very simple, small tasks and operators****" << std::endl << std::endl;
	std::cout << "- these should fail"<< std::endl;
	std::vector<task> tasks;
	task t;
	t.name = "pickup";
	t.parameters["b"] = boost::any(std::string("a"));
	tasks.push_back(t);
	
	cpphop htn;
	declare_operators(htn);
	declare_methods1(htn);
	
	std::vector<task> plan;
	bool res = htn.plan(state1, tasks, plan, 1, 0);
	
	tasks.clear();
	t.name = "pickup";
	t.parameters["b"] = boost::any(std::string("b"));
	tasks.push_back(t);
	
	htn.clear();
	declare_operators(htn);
	declare_methods1(htn);
	plan.clear();
	res = htn.plan(state1, tasks, plan, 1, 0);
	
	std::cout << "- these should succeed" << std::endl;
	tasks.clear();
	t.name = "pickup";
	t.parameters["b"] = boost::any(std::string("c"));
	tasks.push_back(t);
	
	htn.clear();
	declare_operators(htn);
	declare_methods1(htn);
	plan.clear();
	res = htn.plan(state1, tasks, plan, 1, 0);
	
	tasks.clear();
	t.name = "unstack";
	t.parameters["b"] = boost::any(std::string("a"));
	t.parameters["c"] = boost::any(std::string("b"));
	tasks.push_back(t);
	
	htn.clear();
	declare_operators(htn);
	declare_methods1(htn);
	plan.clear();
	res = htn.plan(state1, tasks, plan, 1, 0);
	
	tasks.clear();
	t.name = "get";
	t.parameters["b1"] = boost::any(std::string("a"));
	tasks.push_back(t);
	
	htn.clear();
	declare_operators(htn);
	declare_methods1(htn);
	plan.clear();
	res = htn.plan(state1, tasks, plan, 1, 0);
	
	std::cout << "- this should fail" << std::endl;
	
	tasks.clear();
	t.name = "get";
	t.parameters["b1"] = boost::any(std::string("b"));
	tasks.push_back(t);
	
	htn.clear();
	declare_operators(htn);
	declare_methods1(htn);
	plan.clear();
	res = htn.plan(state1, tasks, plan, 1, 0);
	
	std::cout << "- this should succeed" << std::endl;
	
	tasks.clear();
	t.name = "get";
	t.parameters["b1"] = boost::any(std::string("c"));
	tasks.push_back(t);
	
	htn.clear();
	declare_operators(htn);
	declare_methods1(htn);
	plan.clear();
	res = htn.plan(state1, tasks, plan, 1, 0);
}

// second test two-block stacking problems
void second_set_test(){
	state goal1a;
	std::cout << "****second test two-block stacking problems, some conditions on the second goal are not declared, but still, will have to be met****" << std::endl << std::endl;
	goal1a.name = "goal1a";
	std::map<std::string, std::string> pos = boost::assign::map_list_of ("c", "b") ("b", "a") ("a", "table");
	goal1a.variables["pos"] = boost::any(pos);
	std::map<std::string, bool> clear = boost::assign::map_list_of ("c", true) ("b", false) ("a", false);
	goal1a.variables["clear"] = boost::any(clear);
	goal1a.variables["holding"] = boost::any(false);
	
	state goal1b;
	goal1b.name = "goal1b";
	pos = boost::assign::map_list_of ("c", "b") ("b", "a");
	goal1b.variables["pos"] = boost::any(pos);
	
	std::vector<task> tasks;
	task t1;
	t1.name = "move_blocks";
	t1.parameters["goal"] = boost::any(goal1a);
	tasks.push_back(t1);
	
	std::cout << "- These should succeed" << std::endl;
	cpphop htn;
	declare_operators(htn);
	declare_methods1(htn);
	std::vector<task> plan;
	htn.plan(state1, tasks, plan, 1, 0);
	
	tasks.clear();
	task t2;
	t2.name = "move_blocks";
	t2.parameters["goal"] = boost::any(goal1b);
	tasks.push_back(t2);
	
	htn.clear();
	declare_operators(htn);
	declare_methods1(htn);
	plan.clear();
	htn.plan(state1, tasks, plan, 1, 0);
}


void fthreaded(cpphop &htn, state& state, std::vector<task> &tasks, std::vector<task>& result, int verbose, suseconds_t miliseconds){
	return_state_t res = htn.plan(state, tasks, result, verbose, miliseconds);
	switch(res){
		case STATE_TRUE:
			std::cout << "TRUE";
			break;
		case STATE_FALSE:
			std::cout << "FALSE";
			break;
		case STATE_PAUSED:
			std::cout << "PAUSED";
			break;
	}
	std::cout << std::endl;
	sleep(10);
}

// third set
void third_set_test(){
	state state2;
	std::cout << "****third set, another block****" << std::endl << std::endl;
	state2.name = "state2";
	std::map<std::string, std::string> pos = boost::assign::map_list_of("a", "c") ("b", "d") ("c", "table") ("d", "table");
	state2.variables["pos"] = boost::any(pos);
	std::map<std::string, bool> clear = boost::assign::map_list_of("a", true) ("c", false) ("b", true) ("d", false);
	state2.variables["clear"] = boost::any(clear);
	state2.variables["holding"] = boost::any(std::string(""));
	
	state goal2a;
	goal2a.name = "goal2a";
	pos = boost::assign::map_list_of("b", "c") ("a", "d") ("c", "table") ("d", "table");
	goal2a.variables["pos"] = boost::any(pos);
	clear = boost::assign::map_list_of("a", true) ("c", false) ("b", true) ("d", false);
	goal2a.variables["clear"] = boost::any(clear);
	goal2a.variables["holding"] = boost::any(std::string(""));
	
	state goal2b;
	goal2b.name = "goal2b";
	pos = boost::assign::map_list_of("b", "c") ("a", "d") ;
	goal2b.variables["pos"] = boost::any(pos);
	
	std::vector<task> tasks;
	task t1;
	t1.name = "move_blocks";
	t1.parameters["goal"] = boost::any(goal2a);
	tasks.push_back(t1);
	
	std::cout << "- These should succeed" << std::endl;
	cpphop htn;
	declare_operators(htn);
	declare_methods1(htn);
	std::vector<task> plan;
	htn.plan(state2, tasks, plan, 3, 0);
	
	tasks.clear();
	task t2;
	t2.name = "move_blocks";
	t2.parameters["goal"] = boost::any(goal2b);
	tasks.push_back(t2);
	
	htn.clear();
	declare_operators(htn);
	declare_methods1(htn);
	plan.clear();
	//htn.plan(state2, tasks, plan, 3, 0);
	
		
	boost::thread thr(fthreaded, htn, state2, tasks, plan, 3, 1);
	std::cout << "*******THREAD INICIADO!!!" << std::endl;
	if(htn.pause_plan()){
		std::cout << "*******PAUSED!!!" << std::endl;
		sleep(5);
		std::cout << "*******Going to plan again!!!!!" << std::endl;
		htn.plan(state2, tasks, plan, 1, 0);
	}else{
		std::cout << "*******DID NOT PAUSE!!!" << std::endl;
	}
	
}

// fourth_set, bw_large_d from SHOP distribution
void fourth_set_test(){
	std::cout << "****fourth_set, bw_large_d from SHOP distribution****" << std::endl << std::endl;
	state state3;
	state3.name = "state3";
	std::map<std::string, std::string> pos = boost::assign::map_list_of("1", "12") ("12", "13") ("13", "table") ("11", "10") ("10", "5") ("5", "4") ("4", "14") ("14", "15") ("15", "table") ("9", "8") ("8", "7") ("7", "6") ("6", "table") ("19", "18") ("18", "17") ("17", "16") ("16", "3") ("3", "2") ("2", "table");
	state3.variables["pos"] = boost::any(pos);
	std::map<std::string, bool> clear = boost::assign::map_list_of("1", true) ("2", false) ("3", false) ("4", false) ("5", false) ("6", false) ("7", false) ("8", false) ("9", true) ("10", false)("11", true)("12", false)("13", false)("14", false)("15", false)("16", false)("17", false)("18", false)("19", true);
	state3.variables["clear"] = boost::any(clear);
	state3.variables["holding"] = boost::any(std::string(""));
	
	state goal3;
	goal3.name = "goal3";
	pos = boost::assign::map_list_of("15", "13") ("13", "8") ("8", "9") ("9", "4") ("4", "table") ("12", "2") ("2", "3") ("3", "16") ("16", "11") ("11", "7") ("7", "6") ("6", "table");
	goal3.variables["pos"] = boost::any(pos);
	clear = boost::assign::map_list_of("17", true) ("15", true) ("12", true);
	goal3.variables["clear"] = boost::any(clear);
	
	std::vector<task> tasks;
	task t;
	t.name = "move_blocks";
	t.parameters["goal"] = boost::any(goal3);
	tasks.push_back(t);
	
	std::cout << "- This should succeed" << std::endl;
	
	cpphop htn;
	declare_operators(htn);
	declare_methods1(htn);
	std::vector<task> plan;
	/*
	while(htn.plan(state3, tasks, plan, 1, 1) == STATE_PAUSED){
		std::cout << "***** PAUSED!!!!";
	}
	*/
}



void fith_set_test(){
	std::cout << "****backtracking test****" << std::endl << std::endl;
	
	std::vector<task> tasks;
	task t;
	t.name = "get";
	t.parameters["b1"] = boost::any(std::string("a"));
	tasks.push_back(t);
	
	cpphop htn;
	declare_operators(htn);
	declare_methods2(htn);
	
	std::vector<task> plan;
	htn.plan(state1, tasks, plan, 2, 0);
	
	std::cout << "****Now it shouldn't backtrack****" << std::endl << std::endl;
	
	tasks.clear();
	task t2;
	t2.name = "get";
	t2.parameters["b1"] = boost::any(std::string("c"));
	tasks.push_back(t2);
	
	htn.clear();
	declare_operators(htn);
	declare_methods2(htn);
	
	plan.clear();
	htn.plan(state1, tasks, plan, 2, 0);
	
	std::cout << "****Now it should fail****" << std::endl << std::endl;
	
	tasks.clear();
	task t3;
	t3.name = "get";
	t3.parameters["b1"] = boost::any(std::string("b"));
	tasks.push_back(t3);
	
	htn.clear();
	declare_operators(htn);
	declare_methods2(htn);
	
	plan.clear();
	htn.plan(state1, tasks, plan, 2, 0);
}

int main(){
	state1.name = "state1";
	std::map<std::string, std::string> pos = boost::assign::map_list_of("a", "b") ("b", "table") ("c", "table");
	state1.variables["pos"] = boost::any(pos);
	std::map<std::string, bool> clear = boost::assign::map_list_of("a", true) ("b", false) ("c", true);
	state1.variables["clear"] = boost::any(clear);
	state1.variables["holding"] = boost::any(std::string(""));

	first_set_test();
	second_set_test();
	third_set_test();
	fourth_set_test();
	fith_set_test();
	return 0;
}

