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
#include <cstring>
#include <sstream>
#include "SshSession.h"

using namespace std;

#define MAX 100
#define NOMARK		-1
#define REMOVED		-2

#define MAX_UTIL 0.95
#define LIMIT_UTIL 0.9			//use to make decision
#define MIN_ENV_UTIL 0.1
#define MIN_TOTAL_UTIL 0.3

const string topoFile = "topo.txt";
//const string utilFile = "util.txt";
const string apCredentialFile = "ap_credential.csv";
const string statFilename = "statistic.csv";

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

typedef unordered_map<int, ChannelInfo> Domain; //<channelNo, ChannelInfo>


class ChannelSwitching{
private:
	map<int, int> solution;					//a way to assign channel to each AP
	int **adjacent;			//specify if an AP is adjacent to the others
	vector<int> currentChannel;				//current channel of each AP
	vector<Domain> domain;
	SshSession *sshSessions;
	unsigned int nAP;								//number of APs
	long *oldThroughput;
public:
	ChannelSwitching();
	~ChannelSwitching();
	void readTopoFromFile(const string filename);
	void readCurrentChannelFromFile(const string filename);
	void readUtilizationFromFile(const string filename);
	void prepareData(string topoFile, string currentChannelFile, string utilFile);
	void nodeConsistency();
	void arcConsistency();
	bool dwo(int varIndex);									//domain wipe out
	bool searchFC4(list<int> vars, int level);
	bool checkForward4(list<int> vars, int level, int varIndex, int value);
	bool fc4();
	void restore(list<int> vars, int level);
	void printTopo();
	void printChannelInfo();
	void printCurrentChannel();
	void printSolution();
	void readSshInfo(const string filename);
	void importDataFromAp();
	void switchChannel();
	void exportStatistic();
};

int main(int argc, char** argv){
	cout << "Entering main()\n";

	//string topoFile(argv[1], argv[1] + strlen(argv[1]));
	//string currentChannelFile(argv[2], argv[2] + strlen(argv[2]));
	//string utilFile(argv[3], argv[3] + strlen(argv[3]));

	string currentChannelFile, utilFile;

	ChannelSwitching cs;
	cs.prepareData(topoFile, currentChannelFile, utilFile);

	cs.printCurrentChannel();
	cs.printChannelInfo();

	if (cs.fc4()){
		cs.printSolution();
		cs.switchChannel();
		cs.exportStatistic();
	}else{
		cout << "There is no solution\n";
	}
	return 0;
}

ChannelSwitching::ChannelSwitching(){
	cout << "Entering ChannelSwitching::ChannelSwitching()\n";
	adjacent = NULL;
	sshSessions = NULL;
	//prepareData();
	cout << "Leaving ChannelSwitching::ChannelSwitching()\n";
}

ChannelSwitching::~ChannelSwitching(){
	if (adjacent){
		for (unsigned int i = 0; i < nAP; i ++){
			delete[] adjacent[i];
		}
		delete[] adjacent;
	}
	if (sshSessions)
		delete[] sshSessions;
	if (oldThroughput){
		delete[] oldThroughput;
	}
}

/** 
 * @varIndex: variable index
 * Return: true if the domain has been wiped out
 */
bool ChannelSwitching::dwo(int varIndex){
	cout << "Entering dwo(" << varIndex << ")\n";
	Domain *d = &domain[varIndex];
	for (auto it = d->begin(); it != d->end(); ++it){
		ChannelInfo *chanInfo = &(it->second);
		if (chanInfo->mark == NOMARK){
			cout << "Leaving dwo(" << varIndex << "): false" << "\n";
			return false;
		}
	}
	cout << "Leaving dwo(" << varIndex << "): true" << "\n";
	return true;
}

void ChannelSwitching::restore(list<int> vars, int level){
	cout << "Entering restore() at level " << level << '\n';
	/* restore domain to previous state */
	for (auto it = vars.begin(); it != vars.end(); ++it){
		Domain *d = &(domain[*it]);
		for (auto it2 = d->begin(); it2 != d->end(); ++it2){
			if (it2->second.mark == level){
				cout << "restore domain of var " << *it << '\n';
				it2->second.mark = NOMARK;
			}
		}
	}
	cout << "Leaving restore() at level " << level << '\n';
}

bool ChannelSwitching::searchFC4(list<int> vars, int level){
	cout << "Entering searchFC4() at level " << level << '\n';
	unsigned int Vi = vars.back();
	cout << "selected var " << Vi << '\n';
	vars.pop_back();
	Domain *di = &domain[Vi];
	for (auto it = di->begin(); it != di->end(); ++it){
		ChannelInfo *chanInfo = &(it->second);
		int chanNo = it->first;
		if (chanInfo->mark == NOMARK){
			cout << "add (Vi,vi)=(" << Vi << ',' << chanNo << ") to the solution\n";
			solution.insert(pair<int,int>(Vi, chanNo));
			if (Vi == 0){
				/* found a solution */
				cout << "Leaving searchFC4() at level " << level << ": last var\n";
				return true;
			}
			else{
				/* try to achieve partial arc-consistency */
				if (checkForward4(vars, level, Vi, chanNo) &&
					searchFC4(vars, level + 1)){
					cout << "Leaving searchFC4() at level " << level << ": true\n";
					return true;
				}
				else{
					cout << "remove var " << Vi << " from the solution\n";
					solution.erase(Vi);
					restore(vars, level);
				}
			}
		}
	}
	cout << "Leaving searchFC4() at level " << level << ": false\n";
	return false;
}

/**
 * @varIndex: variable instantiated
 * @value:	value assigned to the var at varIndex
 * Return:	true all domains have not been wiped out
 */
bool ChannelSwitching::checkForward4(list<int> vars, int level, int varIndex, int value){
	cout << "Entering checkForward4() at level " << level 
		 << " with (" << varIndex << ',' << value << ")\n";
	list<VarValuePair> *usp = &domain[varIndex][value].unsupportedSet;
	cout << "unsupportedSet: \n";
	for (auto it = usp->begin(); it != usp->end(); ++it){
		int Vj = it->varIndex;
		int vj = it->value;
		cout << "(" << Vj << ',' << vj << "): ";
		ChannelInfo *chanInfo = &domain[Vj][vj];
		bool varsHasVj = find(vars.begin(), vars.end(), Vj) != vars.end();
		if (varsHasVj && chanInfo->mark == NOMARK){
			cout << "marked\n";
			chanInfo->mark = level;
			if (dwo(Vj)){
				cout << "Leaving checkForward4(): false\n";
				return false;
			}
		}
		cout << '\n';
	}
	cout << "Leaving checkForward4(): true\n";
	return true;
}

bool ChannelSwitching::fc4(){
	cout << "Entering fc4()\n";
	cout << "unmask and clear unsupportedSet for all channel\n";
	for (unsigned int i = 0; i < domain.size(); ++i){
		Domain *d = &domain[i];
		for (auto it = d->begin(); it != d->end(); ++it){
			ChannelInfo *chanInfo = &(it->second);
			chanInfo->mark = NOMARK;
			chanInfo->unsupportedSet.clear();
		}
	}
	solution.clear();
	cout << "initialize unsupportedSet of all channels\n";
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
							vvp.varIndex = j;
							vvp.value = it2->first;
							//cout << "(" << i << ',' << it->first << ") usp + "
							//	 << "(" << j << ',' << it2->first << ")\n";
							info1->unsupportedSet.push_back(vvp);

							vvp.varIndex = i;
							vvp.value = it->first;
							//cout << "(" << j << ',' << it2->first << ") usp + "
							//	 << "(" << i << ',' << it->first << ")\n";
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
	nodeConsistency();
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

void ChannelSwitching::prepareData(string topoFile, string currentChannelFile, string utilFile){
	cout << "Entering ChannelSwitching::prepareData()\n";
	readTopoFromFile(topoFile);
	sshSessions = new SshSession[nAP];
	oldThroughput = new long[nAP];
	readSshInfo(apCredentialFile);

	for (unsigned int i = 0; i < nAP; i++){
		sshSessions[i].connectSsh();
	}

	importDataFromAp();

	//readCurrentChannelFromFile(currentChannelFile);
	//readUtilizationFromFile(utilFile);
	//srand(time(NULL));
	//solution.clear();
	//std::default_random_engine totalGenerator;
	//std::uniform_real_distribution<double> totalDistribution(MIN_TOTAL_UTIL, MAX_UTIL);
	//for (unsigned int i = 0; i < nAP; i++){
	//	domain.push_back(Domain());
	//	for (int j = 1; j < 12; j += 5){
	//		ChannelInfo chanInfo;
	//		chanInfo.mark = NOMARK;
	//		/* generate channel utilization randomly
	//		 * total_util:	[0.3, MAX_UTIL] => distribution: uniform
	//		 * env_util:	[0.1, total_util] => distribution: uniform*/
	//		float totalUtil = totalDistribution(totalGenerator);
	//		chanInfo.util.totalUtil = totalUtil;

	//		//std::default_random_engine envGenerator;
	//		//std::uniform_real_distribution<double> envDistribution(MIN_ENV_UTIL, totalUtil);
	//		//chanInfo.util.envUtil = envDistribution(envGenerator);
	//		chanInfo.util.envUtil = MIN_ENV_UTIL + rand()/((float)RAND_MAX/(totalUtil - MIN_ENV_UTIL));

	//		domain[i].insert(domain[i].end(), pair<int, ChannelInfo>(j, chanInfo));
	//	}
	//}
	cout << "Leaving ChannelSwitching::prepareData()\n";
}

void ChannelSwitching::readUtilizationFromFile(const string filename){
	cout << "Entering ChannelSwitching::readUtilizationFromFile()\n";
	
	ifstream myfile(filename);

	if (myfile.is_open()){
		for (unsigned int i = 0; i < nAP; i++){
			domain.push_back(Domain());
			for (int j = 1; j < 12; j+=5){
				ChannelInfo chanInfo;
				chanInfo.mark = NOMARK;
				myfile >> chanInfo.util.envUtil >> chanInfo.util.totalUtil;
				domain[i].insert(domain[i].end(), pair<int, ChannelInfo>(j, chanInfo));
			}
		}
	}

	myfile.close();

	cout << "Leaving ChannelSwitching::readUtilizationFromFile()\n";
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

void ChannelSwitching::printChannelInfo(){
	cout << "Entering ChannelSwitching::printChannelInfo()" << '\n';
	for (unsigned int i = 0; i < nAP; i ++){
		cout << "AP " << i << ":\t";
		Domain *d = &domain[i];
		for (auto it = d->begin(); it != d->end(); ++it){
			ChannelUtilization *c = &(it->second.util);
			/* pattern: (chanNo, mark, envUtil, totalUtil) */
			cout << "(" << it->first << ", " << it->second.mark << ", "
				 << c->envUtil << ", " << c->totalUtil << ")\t";
		}
		cout << '\n';
	}
	cout << "Leaving ChannelSwitching::printChannelInfo()\n";
}

void ChannelSwitching::printSolution(){
	cout << "Entering printSolution()\n";
	for (unsigned int i = 0; i < nAP; i++){
		cout << "AP " << i << ": " << solution[i] << '\n';
	}
	cout << "Leaving printSolution()\n";
}

void ChannelSwitching::printCurrentChannel(){
	cout << "Entering printCurrentChannel()\n";
	for (unsigned int i = 0; i < nAP; i++){
		cout << "AP " << i << ": " << currentChannel[i] << '\n';
	}
	cout << "Leaving printCurrentChannel()\n";
}

void ChannelSwitching::nodeConsistency(){
	cout << "Entering nodeConsistency()\n";
	for (unsigned int i = 0; i < nAP; ++i){
		ChannelUtilization *currentChanUtil = &(domain[i][currentChannel[i]].util);
		float currentBssUtil = currentChanUtil->totalUtil - currentChanUtil->envUtil;
		Domain *d = &(domain[i]);
		for (auto it = d->begin(); it != d->end(); ++it){
			ChannelInfo *info = &(it->second);
			if (currentBssUtil + info->util.envUtil >= LIMIT_UTIL){
				/* mark removed */
				cout << "remove value " << it->first << " from var " << i << '\n';
				info->mark = REMOVED;
			}
		}
	}
	cout << "Leaving nodeConsistency()\n";
}

void ChannelSwitching::readSshInfo(const string filename){
	cout << "Entering ChannelSwitching::readSshInfo()\n";

	ifstream myfile(filename);
	if (myfile.is_open()){
		string line;
		getline(myfile, line);
		for (unsigned int i = 0; i < nAP; i++){
			getline(myfile, line);
			stringstream ss(line);
			string userName, address, portstr, keyfile;
			getline(ss, userName, ',');
			getline(ss, address, ',');
			getline(ss, portstr, ',');
			getline(ss, keyfile, ',');
			int port = stoi(portstr, NULL, 10);
			sshSessions[i].setUserName(userName.c_str())
						  .setAddress(address.c_str())
						  .setPort(port)
						  .setPrivateKeyFile(keyfile.c_str());
		}
	}
	cout << "Leaving ChannelSwitching::readSshInfo()\n";
}

void ChannelSwitching::importDataFromAp(){
	cout << "Entering ChannelSwitching::importDataFromAp()\n";

	ssh_channel *utilChannels= new ssh_channel[nAP];
	ssh_channel *throughputChannels = new ssh_channel[nAP];

	for (unsigned int i = 0; i < nAP; i++){
		utilChannels[i] = sshSessions[i].runCommandAsync("/root/get_chan_util");
		throughputChannels[i] = sshSessions[i].runCommandAsync("/root/get_throughput");
	}

	for (unsigned int i = 0; i < nAP; i++){
		/* utilization data */
		string utilResult = sshSessions[i].getChannelBuffer(utilChannels[i]);
		stringstream ss(utilResult);
		int chan;
		ss >> chan;
		currentChannel.push_back(chan);

		domain.push_back(Domain());
		for (int j = 1; j < 12; j+=5){
			ChannelInfo chanInfo;
			chanInfo.mark = NOMARK;
			ss >> chanInfo.util.envUtil >> chanInfo.util.totalUtil;
			domain[i].insert(domain[i].end(), pair<int, ChannelInfo>(j, chanInfo));
		}

		/* throughput data */
		string throughput = sshSessions[i].getChannelBuffer(throughputChannels[i]);
		oldThroughput[i] = stoi(throughput, NULL, 10);
	}

	delete[] utilChannels;
	delete[] throughputChannels;

	cout << "Leaving ChannelSwitching::importDataFromAp()\n";
}

void ChannelSwitching::switchChannel(){
	cout << "Entering ChannelSwitching::switchChannel()\n";

	for (auto it = solution.begin(); it != solution.end(); it++){
		int ap = it->first;
		int chan = it->second;
		stringstream cmd;
		cmd << "/root/switch_channel " << chan;
		string result = sshSessions[ap].runCommand(cmd.str().c_str());
		cout << "channel switching result: " << result << '\n';
	}

	cout << "Entering ChannelSwitching::switchChannel()\n";
}

void ChannelSwitching::exportStatistic(){
	cout << "Entering ChannelSwitching::exportStatistic()\n";

	bool fileExisted = false;
	if (FILE *file = fopen(statFilename.c_str(), "r")){
		fclose(file);
		fileExisted = true;
	}

	ofstream statFile;
	statFile.open(statFilename, ofstream::app);
	if (!statFile.is_open()){
		cerr << "Cannot open statistic file\n";
		exit(1);
	}

	if (!fileExisted){
		statFile << "ap_id,old_channel,old_avail,old_env,old_bss,old_total,old_throughput,new_channel,new_avail,new_env,new_bss,new_total,new_throughput\n";
	}
	
	ssh_channel *utilChannels = new ssh_channel[nAP];
	ssh_channel *throughputChannels = new ssh_channel[nAP];

	for (unsigned int i = 0; i < nAP; i++){
		utilChannels[i] = sshSessions[i].runCommandAsync("/root/get_current_chan_util");
		throughputChannels[i] = sshSessions[i].runCommandAsync("/root/get_throughput");
	}

	for (unsigned int i = 0; i < nAP; i++){
		string result = sshSessions[i].getChannelBuffer(utilChannels[i]);
		stringstream ss(result);

		int oldChannel = currentChannel[i];
		int newChannel;
		ss >> newChannel;

		ChannelUtilization oldUtil, newUtil;

		oldUtil = domain[i][oldChannel].util;
		ss >> newUtil.envUtil >> newUtil.totalUtil;

		float oldAvail = LIMIT_UTIL - oldUtil.envUtil;
		float newAvail = LIMIT_UTIL - newUtil.envUtil;
		float oldBss = oldUtil.totalUtil - oldUtil.envUtil;
		float newBss = newUtil.totalUtil - newUtil.envUtil;

		string throughput = sshSessions[i].getChannelBuffer(throughputChannels[i]);
		long newThroughput = stoi(throughput, NULL, 10);

		statFile << i << ','
				 << oldChannel << ',' << oldAvail << ',' << oldUtil.envUtil << ',' << oldBss << ',' << oldUtil.totalUtil << ',' << oldThroughput[i] << ','
				 << newChannel << ',' << newAvail << ',' << newUtil.envUtil << ',' << newBss << ',' << newUtil.totalUtil << ',' << newThroughput << '\n';
	}
	
	statFile.close();
	delete[] utilChannels;
	delete[] throughputChannels;
	
	cout << "Leaving ChannelSwitching::exportStatistic()\n";
}
