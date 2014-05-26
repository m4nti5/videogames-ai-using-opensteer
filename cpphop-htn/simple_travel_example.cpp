#include <iostream>
#include "cpphop.hpp"
/*
The "travel from home to the park" example from Dana Nau lectures.
Author: Marcos Grillo <marcos.grillor@gmail.com> 25-05-2014
*/
using namespace cpphophtn;
double taxi_rate(double dist){
	return  (1.5 + 0.5) * dist;
}

bool walk(state &init_state, std::map<std::string, boost::any>& parameters, state &final_state){
	std::string a = boost::any_cast<std::string>(parameters["a"]);
	std::string x = boost::any_cast<std::string>(parameters["x"]);
	std::string y = boost::any_cast<std::string>(parameters["y"]);
	final_state = init_state;
	std::map<std::string, std::string> loc = boost::any_cast< std::map<std::string, std::string> >(final_state.variables["loc"]);
	if(loc[a] == x){
		loc[a] = y;
		final_state.variables["loc"] = boost::any(loc);
		return true;
	}
	return false;
}

bool call_taxi(state &init_state, std::map<std::string, boost::any>& parameters, state &final_state){
	std::string a = boost::any_cast<std::string>(parameters["a"]);
	std::string x = boost::any_cast<std::string>(parameters["x"]);
	final_state = init_state;
	std::map<std::string, std::string> loc = boost::any_cast< std::map<std::string, std::string> >(final_state.variables["loc"]);
	loc["taxi"] = x;
	final_state.variables["loc"] = boost::any(loc);
	return true;
}

bool ride_taxi(state &init_state, std::map<std::string, boost::any>& parameters, state &final_state){
	std::string a = boost::any_cast<std::string>(parameters["a"]);
	std::string x = boost::any_cast<std::string>(parameters["x"]);
	std::string y = boost::any_cast<std::string>(parameters["y"]);
	final_state = init_state;
	std::map<std::string, std::string> loc = boost::any_cast< std::map<std::string, std::string> >(final_state.variables["loc"]);
	if(loc["taxi"] == x && loc[a] == x){
		loc["taxi"] = y;
		loc[a] = y;
		std::map<std::string, double> owe = boost::any_cast< std::map<std::string, double> >(init_state.variables["owe"]);
		std::map< std::string, std::map<std::string, double> > dist = boost::any_cast< std::map< std::string, std::map<std::string, double> > >(init_state.variables["dist"]);
		owe[a] = taxi_rate(boost::any_cast<double>(dist[x][y]));
		final_state.variables["loc"] = boost::any(loc);
		final_state.variables["owe"] = boost::any(owe);
		return true;
	}
	return false;
}

bool pay_driver(state &init_state, std::map<std::string, boost::any>& parameters, state &final_state){
	std::string a = boost::any_cast<std::string>(parameters["a"]);
	final_state = init_state;
	std::map<std::string, double> cash = boost::any_cast< std::map<std::string, double> >(final_state.variables["cash"]);
	std::map<std::string, double> owe = boost::any_cast< std::map<std::string, double> >(final_state.variables["owe"]);
	if(cash[a] >= owe[a]){
		cash[a] -= owe[a];
		owe[a] = 0.0;
		final_state.variables["cash"] = boost::any(cash);
		final_state.variables["owe"] = boost::any(owe);
		return true;
	}
	return false;
}

bool travel_by_foot(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::string a = boost::any_cast<std::string>(parameters["a"]);
	std::string x = boost::any_cast<std::string>(parameters["x"]);
	std::string y = boost::any_cast<std::string>(parameters["y"]);
	std::map< std::string, std::map<std::string, double> > dist = boost::any_cast< std::map< std::string, std::map<std::string, double> > >(init_state.variables["dist"]);
	if(dist[x][y] <= 2){
		task t;
		t.name = "walk";
		t.parameters["a"] = boost::any(std::string(a));
		t.parameters["x"] = boost::any(std::string(x));
		t.parameters["y"] = boost::any(std::string(y));
		tasks.push_back(t);
		return true;
	}
	return false;
}

bool travel_by_taxi(state &init_state, std::map<std::string, boost::any>& parameters, std::vector<task>& tasks){
	std::string a = boost::any_cast<std::string>(parameters["a"]);
	std::string x = boost::any_cast<std::string>(parameters["x"]);
	std::string y = boost::any_cast<std::string>(parameters["y"]);
	std::map<std::string, double> cash = boost::any_cast< std::map<std::string, double> >(init_state.variables["cash"]);
	std::map< std::string, std::map<std::string, double> > dist = boost::any_cast< std::map< std::string, std::map<std::string, double> > >(init_state.variables["dist"]);
	if(cash[a] >= taxi_rate(dist[x][y])){
		task t;
		t.name = "call_taxi";
		t.parameters["a"] = boost::any(std::string(a));
		t.parameters["x"] = boost::any(std::string(x));
		tasks.push_back(t);
		t.name = "ride_taxi";
		t.parameters["y"] = boost::any(std::string(y));
		tasks.push_back(t);
		t.parameters.clear();
		t.name = "pay_driver";
		t.parameters["a"] = boost::any(std::string(a));
		tasks.push_back(t);
		return true;
	}
	return false;
}
/*

state1 = pyhop.State('state1')
state1.loc = {'me':'home'}
state1.cash = {'me':20}
state1.owe = {'me':0}
state1.dist = {'home':{'park':8}, 'park':{'home':8}}

print("""
********************************************************************************
Call pyhop.pyhop(state1,[('travel','me','home','park')]) with different verbosity levels
********************************************************************************
""")
print('INITIAL STATE:')
pyhop.print_state(state1, 3)
print("- If verbose=0 (the default), Pyhop returns the solution but prints nothing.\n")
print pyhop.pyhop(state1,[('travel','me','home','park')], 3)

print('FINAL STATE:')
pyhop.print_state(state1, 2)
"""
print('- If verbose=1, Pyhop prints the problem and solution, and returns the solution:')
pyhop.pyhop(state1,[('travel','me','home','park')],verbose=1)

print('- If verbose=2, Pyhop also prints a note at each recursive call:')
pyhop.pyhop(state1,[('travel','me','home','park')],verbose=2)
print('- If verbose=3, Pyhop also prints the intermediate states:')
pyhop.pyhop(state1,[('travel','me','home','park')],verbose=3)

"""
*/

void simple_test(){

	cpphop htn;
	htn_operator w;
	std::vector<task> plan;
	w.name = "walk";
	w.action = walk;
	htn.declare_operator("walk", w);
	
	w.name = "call_taxi";
	w.action = call_taxi;
	htn.declare_operator("call_taxi", w);
	
	w.name = "ride_taxi";
	w.action = ride_taxi;
	htn.declare_operator("ride_taxi", w);
	
	w.name = "pay_driver";
	w.action = pay_driver;
	htn.declare_operator("pay_driver", w);
	
	std::vector<task> tasks;
	task t;
	t.name = "walk";
	t.parameters["a"] = boost::any(std::string("me"));
	t.parameters["x"] = boost::any(std::string("home"));
	t.parameters["y"] = boost::any(std::string("park"));
	tasks.push_back(t);
	
	state s;
	s.name = "goal";
	std::map<std::string, std::string> loc;
	loc["me"] = "home";
	s.variables["loc"] = boost::any(loc);
	
	htn.plan(s, tasks, plan, 3);
	std::cout << "plan: " << plan << std::endl;
}

void compound_test(){
	state s;
	std::vector<task> plan;
	htn_operator w;
	s.name = "state1";
	
	std::map<std::string, std::string> loc;
	loc["me"] = "home";
	s.variables["loc"] = loc;
	
	std::map<std::string, double> cash;
	cash["me"] = 20.0;
	s.variables["cash"] = cash;
	
	std::map<std::string, double> owe;
	owe["me"] = 0.0;
	s.variables["owe"] = owe;
	
	std::map< std::string, std::map<std::string, double> > dist;
	dist["home"]["park"] = 8.0;
	dist["park"]["home"] = 8.0;
	s.variables["dist"] = dist;
	cpphop htn;
	w.name = "walk";
	w.action = walk;
	htn.declare_operator("walk", w);
	
	w.name = "call_taxi";
	w.action = call_taxi;
	htn.declare_operator("call_taxi", w);
	
	w.name = "ride_taxi";
	w.action = ride_taxi;
	htn.declare_operator("ride_taxi", w);
	
	w.name = "pay_driver";
	w.action = pay_driver;
	htn.declare_operator("pay_driver", w);
	
	method m;
	std::vector<method> mtds;
	m.name = "travel_by_foot";
	m.method_exe = travel_by_foot;
	//mtds.push_back(m);
	m.name = "travel_by_taxi";
	m.method_exe = travel_by_taxi;
	mtds.push_back(m);
	htn.declare_methods("travel",  mtds);
	
	std::vector<task> tasks;
	task t;
	
	t.name = "travel";
	t.parameters["a"] = boost::any(std::string("me"));
	t.parameters["x"] = boost::any(std::string("home"));
	t.parameters["y"] = boost::any(std::string("park"));
	tasks.push_back(t);
	
	htn.plan(s, tasks, plan, 3);
	
}

int main(){
	// simple_test();
	compound_test();
	return 0;
}

