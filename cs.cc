#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <unordered_map>
#include <queue>
#include <map>
#include <set>
#include <list>
#include <algorithm>
#include <random>

using namespace std;

#define MAX 100
#define NOMARK		-1
#define NOEXIST		-2

#define MAX_UTIL 0.95
#define LIMIT_UTIL 0.9			//use to make decision
#define MIN_ENV_UTIL 0.1
#define MIN_TOTAL_UTIL 0.3

const string topoFile = "topo.txt";
const string currentChannelFile = "channels.txt";

struct VarValuePair{
	int varIndex;							//variable index
	int value;								//value of the variable
};

struct ChannelUtilization{
	float totalUtil;
	float envUtil;
};

struct ChannelInfo{
	list<VarValuePair> unsupportedSet;
	int mark;
	ChannelUtilization util;
};

typedef unordered_map<int, ChannelInfo> Domain;


class ChannelSwitching{
private:
	map<int, int> solution;					//a way to assign channel to each AP
	int **adjacent;			//specify if an AP is adjacent to the others
	vector<int> currentChannel;				//current channel of each AP
	vector<Domain> domain;
	unsigned int nAP;								//number of APs
public:
	ChannelSwitching();
	~ChannelSwitching();
	void readTopoFromFile(const string filename);
	void readCurrentChannelFromFile(const string filename);
	void prepareData();
	bool dwo(int varIndex);									//domain wipe out
	bool searchFC4(list<int> vars, int level);
	bool checkForward4(list<int> vars, int level, int varIndex, int value);
	bool fc4();
	void restore(list<int> vars, int level);
	void printTopo();
	void printChannelUtilization();
	void printSolution();
};

int main(){
	ChannelSwitching cs;
	cs.printChannelUtilization();
	return 0;
}

ChannelSwitching::ChannelSwitching(){
	cout << "Entering ChannelSwitching::ChannelSwitching()\n";
	prepareData();
	cout << "Leaving ChannelSwitching::ChannelSwitching()\n";
}

ChannelSwitching::~ChannelSwitching(){
	for (unsigned int i = 0; i < nAP; i ++){
		delete[] adjacent[i];
	}
	delete[] adjacent;
}

bool ChannelSwitching::dwo(int varIndex){
	Domain *d = &domain[varIndex];
	for (auto it = d->begin(); it != d->end(); ++it){
		ChannelInfo *chanInfo = &(it->second);
		if (chanInfo->mark == NOMARK)
			return true;
	}
	return false;
}

void ChannelSwitching::restore(list<int> vars, int level){
	/* restore domain to previous state */
	for (auto it = vars.begin(); it != vars.end(); ++it){
		Domain *d = &(domain[*it]);
		for (auto it2 = d->begin(); it2 != d->end(); ++it2){
			if (it2->second.mark == level)
				it2->second.mark = NOMARK;
		}
	}
}

bool ChannelSwitching::searchFC4(list<int> vars, int level){
	unsigned int Vi = vars.back();
	vars.pop_back();
	Domain *di = &domain[Vi];
	for (auto it = di->begin(); it != di->end(); ++it){
		ChannelInfo *chanInfo = &(it->second);
		int chanNo = it->first;
		if (chanInfo->mark == NOMARK){
			solution.insert(pair<int,int>(Vi, chanNo));
			if (Vi == nAP - 1)
				/* found a solution */
				return true;
			else{
				/* try to achieve partial arc-consistency */
				if (checkForward4(vars, level, Vi, chanNo) &&
					searchFC4(vars, level + 1))
					return true;
				else{
					solution.erase(Vi);
					restore(vars, level);
				}
			}
		}
	}
	return false;
}

bool ChannelSwitching::checkForward4(list<int> vars, int level, int varIndex, int value){
	list<VarValuePair> *usp = &domain[varIndex][value].unsupportedSet;
	for (auto it = usp->begin(); it != usp->end(); ++it){
		int Vj = it->varIndex;
		int vj = it->value;
		ChannelInfo *chanInfo = &domain[Vj][vj];
		bool varsHasVj = find(vars.begin(), vars.end(), Vj) != vars.end();
		if (varsHasVj && chanInfo->mark == NOMARK){
			chanInfo->mark = level;
			if (dwo(Vj))
				return false;
		}
	}
	return true;
}

bool ChannelSwitching::fc4(){
	for (unsigned int i = 0; i < domain.size(); ++i){
		Domain *d = &domain[i];
		for (auto it = d->begin(); it != d->end(); ++it){
			ChannelInfo *chanInfo = &(it->second);
			chanInfo->mark = NOMARK;
			chanInfo->unsupportedSet.clear();
		}
	}
	solution.clear();
	for (unsigned int i = 0; i < nAP; i++){
		for (unsigned int j = i + 1; j < nAP; j++){
			if (adjacent[i][j]){
				Domain *di = &domain[i];
				Domain *dj = &domain[j];
				for (auto it = di->begin(); it != di->end(); ++it){
					for (auto it2 = dj->begin(); it2 != dj->end(); ++it2){
						if (it->first == it2->first){
							ChannelInfo *info1 = &it->second;
							ChannelInfo *info2 = &it2->second;

							VarValuePair vvp;
							vvp.varIndex = i;
							vvp.value = it->first;
							info1->unsupportedSet.push_back(vvp);

							vvp.varIndex = j;
							vvp.value = it2->first;
							info2->unsupportedSet.push_back(vvp);
						}
					}
				}
			}
		}
	}

	list<int> vars;
	for (unsigned int i = 0; i < domain.size(); ++i){
		vars.push_back(i);
	}
	return searchFC4(vars, 1);
}

void ChannelSwitching::readTopoFromFile(const string filename){
	cout << "Entering ChannelSwitching::readTopoFromFile()\n";
	nAP = 0;
	cout << "Clear old data done\n";
	ifstream myfile(filename);
	cout << "Open file done\n";
	if (myfile.is_open()){
		cout << "File opened\n";
		myfile >> nAP;
		adjacent = new int*[nAP];
		cout << "Read nAP done\n";
		for (unsigned int i = 0; i < nAP; i++){
			adjacent[i] = new int[nAP];
			for (unsigned j = 0; j < nAP; j++){
				myfile >> adjacent[i][j];
			}
		}
		cout << "Read file done\n";
		myfile.close();
		cout << "File closed\n";
	}else{
		cout << "unable to open file " << filename << "\n";
		exit(1);
	}
	cout << "Leaving ChannelSwitching::readTopoFromFile()\n";
}

void ChannelSwitching::readCurrentChannelFromFile(const string filename){
	cout << "Entering ChannelSwitching::readCurrentChannelFromFile()\n";

//	if (currentChannel.size())
//		currentChannel.clear();

	cout << "Clear old data done\n";
	ifstream myfile(filename);
	cout << "Open file done\n";
	if (myfile.is_open()){
		cout << "File opened\n";
		for (unsigned int i = 0; i < nAP; i++){
			int c;
			myfile >> c;
			currentChannel.push_back(c);
		}
		myfile.close();
		cout << "File closed\n";
	}else{
		cout << "unable to open file " << filename << "\n";
	}
	cout << "Leaving ChannelSwitching::readCurrentChannelFromFile()\n";
}

void ChannelSwitching::prepareData(){
	cout << "Entering ChannelSwitching::prepareData()\n";
	srand(time(NULL));
	readTopoFromFile(topoFile);
	readCurrentChannelFromFile(currentChannelFile);
	solution.clear();
	std::default_random_engine totalGenerator;
	std::uniform_real_distribution<double> totalDistribution(MIN_TOTAL_UTIL, MAX_UTIL);
	for (unsigned int i = 0; i < nAP; i++){
		domain.push_back(Domain());
		for (int j = 1; j < 12; j += 5){
			ChannelInfo chanInfo;
			chanInfo.mark = NOMARK;
			/* generate channel utilization randomly
			 * total_util:	[0.3, MAX_UTIL] => distribution: uniform
			 * env_util:	[0.1, total_util] => distribution: uniform*/
			float totalUtil = totalDistribution(totalGenerator);
			chanInfo.util.totalUtil = totalUtil;

			//std::default_random_engine envGenerator;
			//std::uniform_real_distribution<double> envDistribution(MIN_ENV_UTIL, totalUtil);
			//chanInfo.util.envUtil = envDistribution(envGenerator);
			chanInfo.util.envUtil = MIN_ENV_UTIL + rand()/((float)RAND_MAX/(totalUtil - MIN_ENV_UTIL));

			domain[i].insert(domain[i].end(), pair<int, ChannelInfo>(j, chanInfo));
		}
	}
	cout << "Leaving ChannelSwitching::prepareData()\n";
}

void ChannelSwitching::printTopo(){
	cout << "Entering ChannelSwitching::printTopo()\n";
	unsigned int n = nAP;
	cout << "Adjacency ---------------\n";
	for (unsigned int i = 0; i < n; i ++){
		for (unsigned int j = 0; j < n; j ++){
			cout << adjacent[i][j] << ' ';
		}
		cout << '\n';
	}
	cout << "Leaving ChannelSwitching::printTopo\n";
}

void ChannelSwitching::printChannelUtilization(){
	cout << "Entering ChannelSwitching::printChannelUtilization()" << '\n';
	for (unsigned int i = 0; i < nAP; i ++){
		cout << "AP " << i << ":\t";
		Domain *d = &domain[i];
		for (auto it = d->begin(); it != d->end(); ++it){
			ChannelUtilization *c = &(it->second.util);
			cout << "(" << it->first << ", "
				 << c->envUtil << ", " << c->totalUtil << ")\t";
		}
		cout << '\n';
	}
	cout << "Leaving ChannelSwitching::printChannelUtilization()\n";
}

void ChannelSwitching::printSolution(){
	
}
