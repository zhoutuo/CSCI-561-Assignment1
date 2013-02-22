#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <queue>
#include <vector>
#include <set>
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
	bool operator () (const float& left, const float& right) const {
		return (abs(left - right) > (1e-5)) && (left < right);
	}
};

typedef set<float, classcmp> speed_set;
typedef map<float, Point, classcmp> speed_map;
typedef vector<float> speed_arr;
typedef vector<Point> speed_arr_content;
//global variables
ofstream output;
Point start, goal;
unsigned int beamK = 10;
char maze[100][100];
speed_set flags[100][100];
speed_map parentTable[100][100];

speed_arr speeds[100][100];
speed_arr_content contents[100][100];

int delX[4] = { 0, -1, 0, 1 };
int delY[4] = { -1, 0, 1, 0 };

template<class T>
string fromNumber(T n) {
	ostringstream ss;
	ss << n;
	return ss.str();
}

bool getFlag(Point point) {
	speed_set& cur = flags[point.y][point.x];
	if(cur.find(point.speed) == cur.end()) {
		return false;
	} else {
		return true;
	}
}

void setFlag(Point point) {
	speed_set& cur = flags[point.y][point.x];
    cur.insert(point.speed);
}

void setParent(Point child, Point Parent) {
    speed_arr& cur = speeds[child.y][child.x];
    speed_arr_content& con = contents[child.y][child.x];

    cur.push_back(child.speed);
    con.push_back(Parent);
}

bool findParent(Point child) {
    speed_arr& cur = speeds[child.y][child.x];
    float epison = 1e-5;
    for(unsigned int i = 0; i < cur.size(); ++i) {
    	if(abs(child.speed - cur[i]) < epison) {
    		return true;
    	}
    }
    return false;
}


Point getParent(Point child) {
    speed_arr& cur = speeds[child.y][child.x];
    float epison = 1e-5;
    for(unsigned int i = 0; i < cur.size(); ++i) {
    	if(abs(child.speed - cur[i]) < epison) {
    		return contents[child.y][child.x][i];
    	}
    }
    return Point();
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
	searchLog += "index ";
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
	setFlag(start);
	Point zero;
	zero.x = zero.y = zero.step = 0;
	zero.speed = 0.0f;
	setParent(start, zero);
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

				logChild(searchLog, indexOfChildren, newPos, false);
				++indexOfChildren;
				if (findParent(newPos)) {
					continue;
				}
				setParent(newPos, cur);
				switch (sign) {
				case 'M':
					newPos.speed -= 0.1;
					/* no break */
				case ' ':
					que.push_back(newPos);
					break;
				case 'G':
					//logPathLength(pathLog, newPos, iteration, false);
					//dump to output file
					isFound = true;
					break;
				}

			}
		}
		logFrontier(searchLog, que);
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
	setFlag(start);
	Point zero;
	zero.x = zero.y = zero.step = 0;
	setParent(start, zero);
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

				logChild(searchLog, indexOfChildren, newPos, false);
				++indexOfChildren;

				if (getFlag(newPos)) {
					continue;
				}
				setParent(newPos, cur);
				switch (sign) {
				case 'M':
					newPos.speed -= 0.1;
					/* no break */
				case ' ':

					setFlag(newPos);
					stack.push_back(newPos);
					break;
				case 'G':
					logPathLength(pathLog, newPos, iteration, false);
					isFound = true;
					break;

				}

			}
		}
		logFrontier(searchLog, stack);
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

void Astar(bool isH1) {

	priority_queue<Point> que;
	AstarState stateMap[100][100];
	for (int i = 0; i < 100; ++i) {
		for (int j = 0; j < 100; ++j) {
			stateMap[i][j] = UNEXPOLDED;
		}
	}
	start.h = distance(start, isH1) / start.speed;
	que.push(start);
	stateMap[start.y][start.x] = EXPLODING;

	Point zero;
	zero.x = zero.y = zero.step = 0;
	setParent(start, zero);
	string searchLog = "Search log\n";
	string pathLog = "";
	int iteration = 1;
	bool isFound = false;

	while (!isFound && !que.empty()) {
		Point cur = que.top();
		que.pop();
		stateMap[cur.y][cur.x] = EXPLODED;
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
				newPos.h = distance(newPos, isH1) / newPos.speed;
				++newPos.step;

				AstarState curState = stateMap[newPos.y][newPos.x];

				if (sign == '*') {
					continue;
				}

				logChild(searchLog, indexOfChildren, newPos, true);
				++indexOfChildren;
				if (curState == EXPLODED) {
					continue;
				} else if (curState == EXPLODING) {
					priority_queue<Point> tmp = que;
					priority_queue<Point> emp;
					while (true) {
						Point top = tmp.top();
						tmp.pop();
						if (newPos.x == top.x && newPos.y == top.y) {
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
				switch (sign) {
				case 'M':
					newPos.speed -= 0.1;
					newPos.h = distance(newPos, isH1) / newPos.speed;
					/* no break */
				case ' ':
					stateMap[newPos.y][newPos.x] = EXPLODING;
					que.push(newPos);
					break;
				case 'G':
					logPathLength(pathLog, newPos, iteration, true);
					//dump to output file
					isFound = true;
					break;
				}

			}
		}
		logFrontier(searchLog, que);
		++iteration;
	}

	if (isFound) {
		output << pathLog;
		output << searchLog;
	}

}

void BS(bool isH1) {

	priority_queue<Point> frontier;

	start.h = distance(start, isH1) / start.speed;
	setFlag(start);
	frontier.push(start);

	Point zero;
	zero.x = zero.y = zero.step = 0;
	setParent(start, zero);
	string searchLog = "Search log\n";
	string pathLog = "";
	int iteration = 1;
	bool isFound = false;

	while (!isFound && !frontier.empty()) {
		Point cur = frontier.top();
		frontier.pop();
		logIteration(searchLog, iteration, cur, true);

		if (cur.speed > 0) {
			int indexOfChildren = 0;
			for (int j = 0; j < 4; ++j) {
				int newX = cur.x + delX[j];
				int newY = cur.y + delY[j];
				Point child = cur;
				char sign = maze[newY][newX];
				child.x = newX;
				child.y = newY;
				child.g += 1 / child.speed;
				child.h = distance(child, isH1) / child.speed;
				++child.step;

				if (sign == '*') {
					continue;
				}

				logChild(searchLog, indexOfChildren, child, false);
				++indexOfChildren;

				if (getFlag (child)) {
					continue;
				}
				setParent(child, cur);
				switch (sign) {
				case 'M':
					child.speed -= 0.1;
					child.h = distance(child, isH1) / child.speed;

					/* no break */
				case ' ':
					setFlag(child);
					frontier.push(child);
					if (frontier.size() > beamK) {
						priority_queue<Point> tmp;
						int t = beamK;
						while (t--) {
							tmp.push(frontier.top());
							frontier.pop();
						}
						frontier = tmp;
					}
					break;
				case 'G':

					logPathLength(pathLog, child, iteration, true);
					//dump to output file
					isFound = true;
					break;
				}

			}
		}

		logFrontier(searchLog, frontier);
		++iteration;
	}

	if (isFound) {
		output << pathLog;
		output << searchLog;
	}
}

int main(int argc, char **argv) {

	clock_t startT = clock();
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
	start.g = 0.0f;

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

	cout << (float) (clock() - startT) / CLOCKS_PER_SEC << endl;
}
