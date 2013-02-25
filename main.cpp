#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <queue>
#include <vector>
#include <map>
#include <ctime>
using namespace std;

struct Point {
	int x;
	int y;
	float speed;
	float g;
	float h;
	int step;

	bool operator <(const Point& other) const {
		return (g + h) > (other.g + other.h);
	}
};

struct classcmp {
	bool operator()(const float& left, const float& right) const {
		return (abs(left - right) > (1e-5)) && (left < right);
	}
};

typedef map<float, Point, classcmp> speed_map;

//global variables
ofstream output;
Point start, goal;
unsigned int beamK = 10;
char maze[100][100];
speed_map parentTable[100][100];
int delX[4] = { 0, 1, 0, -1 };
int delY[4] = { -1, 0, 1, 0 };

template<class T>
string fromNumber(T n) {
	ostringstream ss;
	ss << n;
	return ss.str();
}

void setParent(Point child, Point Parent) {
	speed_map& cur = parentTable[child.y][child.x];
	cur[child.speed] = Parent;
}

bool findParent(Point child) {
	speed_map& cur = parentTable[child.y][child.x];
	if (cur.find(child.speed) == cur.end()) {
		return false;
	} else {
		return true;
	}
}

Point getParent(Point child) {
	speed_map& cur = parentTable[child.y][child.x];
	return cur[child.speed];
}

void logPoint(string& searchLog, Point cur, bool outputH) {
	searchLog += "x = ";
	searchLog += fromNumber(cur.x);
	searchLog += " y = ";
	searchLog += fromNumber(cur.y);
	searchLog += " speed = ";
	searchLog += fromNumber(cur.speed);
	searchLog += " g = ";
	searchLog += fromNumber(cur.g);
	if (outputH) {
		searchLog += " f = ";
		searchLog += fromNumber(cur.g + cur.h);
	}
	searchLog += '\n';
}

void logIteration(string& searchLog, int iteration, Point cur, bool outputH) {
	//logging
	if (iteration <= 100) {
		searchLog += "Iteration = ";
		searchLog += fromNumber(iteration);
		searchLog += '\n';
		searchLog += "Current Node : ";
		logPoint(searchLog, cur, outputH);
		searchLog += "Child List:\n";
	}
}

void logChild(string& searchLog, int index, Point cur, bool outputH) {
	searchLog += "index = ";
	searchLog += fromNumber(index);
	searchLog += ' ';
	logPoint(searchLog, cur, outputH);
}

void logFrontier(string& searchLog, deque<Point>& que) {
	searchLog += "Frontier List:\n";
	for (unsigned int i = 0; i < que.size(); ++i) {
		logChild(searchLog, i, que[i], false);
	}
	searchLog += '\n';
}

void logFrontier(string& searchLog, vector<Point>& stack) {
	searchLog += "Frontier List:\n";
	for (unsigned int i = 0; i < stack.size(); ++i) {
		logChild(searchLog, i, stack[stack.size() - i - 1], false);
	}
	searchLog += '\n';
}

void logFrontier(string& searchLog, priority_queue<Point>& que) {
	searchLog += "Frontier List:\n";
	priority_queue<Point> tmp = que;
	int index = 0;
	while (!tmp.empty()) {
		Point cur = tmp.top();
		tmp.pop();
		logChild(searchLog, index++, cur, true);
	}
	searchLog += '\n';
}

void logPath(string& pathLog, Point cur, bool outputH) {
	if (cur.step != 1) {
		Point parent = getParent(cur);
		logPath(pathLog, parent, outputH);
	}
	logPoint(pathLog, cur, outputH);
}

void logPathLength(string& pathLog, Point goal, int iteration, bool outputH) {
	//find the goal
	pathLog += "Number of Iteration = ";
	pathLog += fromNumber(iteration);
	pathLog += '\n';
	pathLog += "Path length = ";
	pathLog += fromNumber(goal.step);
	pathLog += "\n";
	logPath(pathLog, goal, outputH);
	pathLog += "----------------------------------\n";
}

void BFS() {
	deque<Point> que;
	que.push_back(start);
	string searchLog = "Search log\n";
	string pathLog = "";
	int iteration = 1;
	bool isFound = false;
	while (!isFound && !que.empty()) {
		Point cur = que.front();
		que.pop_front();
		int x = cur.x;
		int y = cur.y;
		logIteration(searchLog, iteration, cur, false);
		if (cur.speed > 0) {
			int indexOfChildren = 0;
			for (int i = 0; i < 4; ++i) {
				int newX = x + delX[i];
				int newY = y + delY[i];
				char sign = maze[newY][newX];
				Point newPos = cur;
				newPos.x = newX;
				newPos.y = newY;
				newPos.g += 1 / newPos.speed;
				++newPos.step;
				if (sign == '*') {
					continue;
				}
				if (iteration <= 100) {
					logChild(searchLog, indexOfChildren, newPos, false);
				}
				++indexOfChildren;
				if (sign == 'M') {
					newPos.speed -= 0.1;
				}
				if (findParent(newPos)) {
					continue;
				} else {
					setParent(newPos, cur);
				}
				if (sign == 'G') {
					logPathLength(pathLog, newPos, iteration, false);
					isFound = true;
				} else {
					que.push_back(newPos);
				}
			}
		}
		if (iteration <= 100) {
			logFrontier(searchLog, que);
		}
		++iteration;
	}
	if (isFound) {
		output << pathLog;
		output << searchLog;
	}
}

void DFS() {
	vector<Point> stack;
	stack.push_back(start);
	string searchLog = "Search log\n";
	string pathLog = "";
	int iteration = 1;
	bool isFound = false;
	while (!isFound && !stack.empty()) {
		Point cur = stack.back();
		stack.pop_back();
		int x = cur.x;
		int y = cur.y;
		logIteration(searchLog, iteration, cur, false);
		if (cur.speed > 0) {
			int indexOfChildren = 0;
			for (int i = 0; i < 4; ++i) {
				int newX = x + delX[i];
				int newY = y + delY[i];
				char sign = maze[newY][newX];
				Point newPos = cur;
				newPos.x = newX;
				newPos.y = newY;
				newPos.g += 1 / newPos.speed;
				++newPos.step;
				if (sign == '*') {
					continue;
				}
				if (iteration <= 100) {
					logChild(searchLog, indexOfChildren, newPos, false);
				}
				++indexOfChildren;
				if (sign == 'M') {
					newPos.speed -= 0.1;
				}
				if (findParent(newPos)) {
					continue;
				} else {
					setParent(newPos, cur);
				}
				if (sign == 'G') {
					logPathLength(pathLog, newPos, iteration, false);
					isFound = true;
				} else {
					stack.push_back(newPos);
				}
			}
		}
		if (iteration <= 100) {
			logFrontier(searchLog, stack);
		}
		++iteration;
	}
	if (isFound) {
		output << pathLog;
		output << searchLog;
	}
}

float distance(Point cur, bool isH1) {
	int delX = cur.x - goal.x;
	int delY = cur.y - goal.y;
	if (isH1) {
		return abs(delX) + abs(delY);
	} else {
		return sqrt(delX * delX + delY * delY);
	}
}

enum AstarState {
	UNEXPOLDED, EXPLODED, EXPLODING
};

typedef map<float, AstarState, classcmp> amap;

void Astar(bool isH1) {
	priority_queue<Point> que;
	amap stateMap[100][100];
	start.h = distance(start, isH1) / start.speed;
	que.push(start);
	stateMap[start.y][start.x][start.speed] = EXPLODING;
	string searchLog = "Search log\n";
	string pathLog = "";
	int iteration = 1;
	bool isFound = false;
	while (!isFound && !que.empty()) {
		Point cur = que.top();
		que.pop();
		stateMap[cur.y][cur.x][cur.speed] = EXPLODED;
		int x = cur.x;
		int y = cur.y;
		logIteration(searchLog, iteration, cur, true);
		if (cur.speed > 0) {
			int indexOfChildren = 0;
			for (int i = 0; i < 4; ++i) {
				int newX = x + delX[i];
				int newY = y + delY[i];
				char sign = maze[newY][newX];
				Point newPos = cur;
				newPos.x = newX;
				newPos.y = newY;
				newPos.g += 1 / newPos.speed;
				++newPos.step;
				if (sign == 'M') {
					newPos.speed -= 0.1;
				}
				newPos.h = distance(newPos, isH1) / newPos.speed;

				if (sign == '*') {
					continue;
				}
				amap& curMap = stateMap[newPos.y][newPos.x];

				//if it has not been found yet
				if (curMap.find(newPos.speed) == curMap.end()) {
					curMap[newPos.speed] = UNEXPOLDED;
				}
				AstarState& curState = curMap[newPos.speed];
				if (iteration <= 100) {
					logChild(searchLog, indexOfChildren, newPos, true);
				}
				++indexOfChildren;
				if (curState == EXPLODED) {
					continue;
				} else if (curState == EXPLODING) {
					priority_queue<Point> tmp = que;
					priority_queue<Point> emp;
					while (true) {
						Point top = tmp.top();
						tmp.pop();
						if (newPos.x == top.x && newPos.y == top.y
								&& (abs(newPos.speed - top.speed) < (1e-5))) {
							if (newPos.g < top.g) {
								setParent(newPos, cur);
								emp.push(newPos);
								while (!tmp.empty()) {
									emp.push(tmp.top());
									tmp.pop();
								}
								que = emp;
							}
							break;
						} else {
							emp.push(top);
						}
					}
					continue;
				}

				setParent(newPos, cur);
				if (sign == 'G') {
					logPathLength(pathLog, newPos, iteration, true);
					isFound = true;
				} else {
					curState = EXPLODING;
					que.push(newPos);
				}
			}
		}
		if (iteration <= 100) {
			logFrontier(searchLog, que);
		}
		++iteration;
	}
	if (isFound) {
		output << pathLog;
		output << searchLog;
	}
}

void BS(bool isH1) {
	priority_queue<Point> que;
	amap stateMap[100][100];
	start.h = distance(start, isH1) / start.speed;
	que.push(start);
	stateMap[start.y][start.x][start.speed] = EXPLODING;
	string searchLog = "Search log\n";
	string pathLog = "";
	int iteration = 1;
	bool isFound = false;
	while (!isFound && !que.empty()) {
		Point cur = que.top();
		que.pop();
		stateMap[cur.y][cur.x][cur.speed] = EXPLODED;
		int x = cur.x;
		int y = cur.y;
		logIteration(searchLog, iteration, cur, true);
		if (cur.speed > 0) {
			int indexOfChildren = 0;
			for (int i = 0; i < 4; ++i) {
				int newX = x + delX[i];
				int newY = y + delY[i];
				char sign = maze[newY][newX];
				Point newPos = cur;
				newPos.x = newX;
				newPos.y = newY;
				newPos.g += 1 / newPos.speed;
				++newPos.step;
				if (sign == 'M') {
					newPos.speed -= 0.1;
				}
				newPos.h = distance(newPos, isH1) / newPos.speed;

				if (sign == '*') {
					continue;
				}
				amap& curMap = stateMap[newPos.y][newPos.x];

				//if it has not been found yet
				if (curMap.find(newPos.speed) == curMap.end()) {
					curMap[newPos.speed] = UNEXPOLDED;
				}
				AstarState& curState = curMap[newPos.speed];
				if (iteration <= 100) {
					logChild(searchLog, indexOfChildren, newPos, true);
				}
				++indexOfChildren;
				if (curState == EXPLODED) {
					continue;
				} else if (curState == EXPLODING) {
					priority_queue<Point> tmp = que;
					priority_queue<Point> emp;
					while (true) {
						Point top = tmp.top();
						tmp.pop();
						if (newPos.x == top.x && newPos.y == top.y
								&& (abs(newPos.speed - top.speed) < (1e-5))) {
							if (newPos.g < top.g) {
								setParent(newPos, cur);
								emp.push(newPos);
								while (!tmp.empty()) {
									emp.push(tmp.top());
									tmp.pop();
								}
								que = emp;
							}
							break;
						} else {
							emp.push(top);
						}
					}
					continue;
				}

				setParent(newPos, cur);

				if (sign == 'G') {
					logPathLength(pathLog, newPos, iteration, true);
					isFound = true;
				} else {
					curState = EXPLODING;
					que.push(newPos);
					if (que.size() > beamK) {
						priority_queue<Point> tmp;
						int t = beamK;
						while (t--) {
							tmp.push(que.top());
							que.pop();
						}

						while (!que.empty()) {
							Point top = que.top();
							stateMap[top.y][top.x][top.speed] = EXPLODED;
							que.pop();
						}

						que = tmp;
					}
				}
			}
		}
		if (iteration <= 100) {
			logFrontier(searchLog, que);
		}
		++iteration;
	}
	if (isFound) {
		output << pathLog;
	}
	output << searchLog;
}

int main(int argc, char **argv) {
	//clock_t startT = clock();
	string error = "arguments of the program are wrong!!!";
	bool errorFlag = false;
	int functionIndex;
	ifstream input;
	int width;
	int height;
	string tmp;
	if (argc == 8) {
		if (strcmp(argv[1], "BFS") == 0) {
			functionIndex = 0;
		} else if (strcmp(argv[1], "DFS") == 0) {
			functionIndex = 1;
		} else if (strcmp(argv[1], "Astar") == 0) {
			functionIndex = 2;
		} else if (strcmp(argv[1], "Beam") == 0) {
			functionIndex = 3;
		} else {
			errorFlag = true;
		}
		if (!errorFlag) {
			start.speed = atof(argv[3]);
			input.open(argv[5]);
			output.open(argv[7]);
		}
	} else if (argc == 10) {
		if (strcmp(argv[1], "Beam") == 0) {
			functionIndex = 3;
			beamK = atoi(argv[3]);
			start.speed = atof(argv[5]);
			input.open(argv[7]);
			output.open(argv[9]);
		} else {
			errorFlag = true;
		}
	} else {
		errorFlag = true;
	}
	if (errorFlag) {
		cout << error << endl;
		return -1;
	}
	//get width
	getline(input, tmp);
	stringstream buffer(tmp);
	buffer >> tmp;
	buffer >> width;
	//get height
	getline(input, tmp);
	stringstream buffer2(tmp);
	buffer2 >> tmp;
	buffer2 >> height;
	//create maze
	for (int i = 0; i < height; ++i) {
		getline(input, tmp);
		for (int j = 0; j < width; ++j) {
			char cur = tmp[j];
			maze[i][j] = cur;
			if (cur == 'S') {
				start.x = j;
				start.y = i;
			} else if (cur == 'G') {
				goal.x = j;
				goal.y = i;
			}
		}
	}
	start.step = 1;
	start.g = 0.0f;
	Point zero;
	zero.x = zero.y = zero.step = 0;
	zero.speed = 0.0f;
	setParent(start, zero);
	switch (functionIndex) {
	case 0:
		//bfs
		BFS();
		break;
	case 1:
		//dfs
		DFS();
		break;
	case 2:
		//astar
		Astar(true);
		break;
	default:
		//beam
		BS(true);
		break;
	}
	input.close();
	output.close();
	//cout << (float) (clock() - startT) / CLOCKS_PER_SEC << endl;
}
