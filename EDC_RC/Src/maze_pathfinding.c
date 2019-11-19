#include "maze_pathfinding.h"

#define INF_MAX 1000
#define UNVISITED 0
#define VISITED 1
#define ARRIVED 0
#define TURNLEFT 14
#define TURNRIGHT 15
#define TURNBACK 16
#define MOVEFORE 11
enum DIRECTION{//maze里的绝对方向
	UP = 1, LEFT, DOWN, RIGHT
};

struct mazenode{
	char wall;	//0-3位分别是上，左，下，右侧有无障碍，高位=0
	
	//路径规划中使用的变量
	struct mazenode* parent;
	int priority;
	char visited;
};

//定义迷宫二维数组（含边界外的一层）
struct mazenode maze[8][8];

//添加墙壁（坐标位置，上0/左1/下2/右3），相邻位置不需要重复添加
void AddWall (char x, char y, char side){
	switch(side){
	case UP:
		maze[y][x].wall |= 1;
		maze[y + 1][x].wall |= 4;
		break;
	case LEFT:
		maze[y][x].wall |= 2;
		maze[y][x - 1].wall |= 8;
		break;
	case DOWN:
		maze[y][x].wall |= 4;
		maze[y - 1][x].wall |= 1;
		break;
	case RIGHT:
		maze[y][x].wall |= 8;
		maze[y][x + 1].wall |= 2;
		break;
	default: break;
	}
}

//将墙壁信息全部清空
void ClearWalls (void){
	for (int i = 0; i < 8; i++){
		for (int j = 0; j < 8; j++){
			maze[i][j].wall = 0;
			//加入边界围墙信息
			if (j == 1) maze[i][j].wall |= 2;
			if (j == 6) maze[i][j].wall |= 8;
			if (i == 1) maze[i][j].wall |= 4;
			if (i == 6) maze[i][j].wall |= 1;
		}
	}
	//添加场地分界挡板信息
	for (int i = 1; i < 6; i++){
		AddWall(i, 6 - i, UP);
		AddWall(i, 6 - i, RIGHT);
	}
}

//参数：当前方向，xy坐标，目标位置xy坐标（范围1-6,要在原坐标基础上加1），当前位置障碍情况（0-2位分别是右，前，左侧有无障碍）
unsigned char PathFinding (char currentDir, char posx, char posy, char aimx, char aimy, char wallrefresh){
	unsigned char decision = 255;

//起点终点重合返回ARRIVED = 0
	if (posx == aimx && posy == aimy) return ARRIVED;

//更新墙壁信息
	switch(currentDir){
	case RIGHT:
		if (wallrefresh & 4) AddWall(posx,posy,UP);
		if (wallrefresh & 2) AddWall(posx,posy,RIGHT);
		if (wallrefresh & 1) AddWall(posx,posy,DOWN);
		break;
	case UP:
		if (wallrefresh & 4) AddWall(posx,posy,LEFT);
		if (wallrefresh & 2) AddWall(posx,posy,UP);
		if (wallrefresh & 1) AddWall(posx,posy,RIGHT);
		break;
	case DOWN:
		if (wallrefresh & 4) AddWall(posx,posy,RIGHT);
		if (wallrefresh & 2) AddWall(posx,posy,DOWN);
		if (wallrefresh & 1) AddWall(posx,posy,LEFT);
		break;
	case LEFT:
		if (wallrefresh & 4) AddWall(posx,posy,DOWN);
		if (wallrefresh & 2) AddWall(posx,posy,LEFT);
		if (wallrefresh & 1) AddWall(posx,posy,UP);
		break;
	default: break;
	}

//清空规划路线的中间信息
	for (int i = 0; i < 8; i++){
		for (int j = 0; j < 8; j++){
		    maze[i][j].parent = 0;
		    maze[i][j].priority = INF_MAX;	//初始优先级置高
			maze[i][j].visited = UNVISITED;
		}
	}

//Dijkstra规划路线
	maze[posy][posx].priority = 0;
	struct mazenode* selectednode = 0;
	struct mazenode* old_snode = 0;
	while (1){
		//选出未访问节点中的最高优先级加入
		int shortest = INF_MAX;
		for (int i = 1; i < 7; i++){
			for (int j = 1; j < 7; j++){
				if (maze[i][j].visited == UNVISITED && maze[i][j].priority < shortest){
					shortest = maze[i][j].priority;
					selectednode = &maze[i][j];
				}
			}
		}
		if (old_snode == selectednode) return 255;	//死循环return
		selectednode -> visited = VISITED;
		//当目标点被选中时循环终止
		if (maze[aimy][aimx].visited == VISITED) break;
		//对此节点的邻居优先级做更新
		struct mazenode* rightnode = (selectednode + 1);
		struct mazenode* leftnode  = (selectednode - 1);
		struct mazenode* downnode  = (selectednode - 8);
		struct mazenode* upnode    = (selectednode + 8);
		//dirjudge
		int selectindex = selectednode - &maze[0][0];
		int parentindex = (selectednode -> parent) ? selectednode -> parent - &maze[0][0] : selectindex;
		int dirjudge = selectindex - parentindex;
		int pathweight = INF_MAX;
		//right
		if ((selectednode -> wall & 8) != 0) pathweight = INF_MAX;
		else if (dirjudge == 0 || dirjudge == 1) pathweight = 1;
		else if (dirjudge == -1) pathweight = 3;
		else  pathweight = 2;
		if (selectednode -> priority + pathweight < rightnode -> priority){
			rightnode -> priority = selectednode -> priority + pathweight;
			rightnode -> parent = selectednode;
		}
		//left
		if ((selectednode -> wall & 2) != 0) pathweight = INF_MAX;
		else if (dirjudge == 0 || dirjudge == -1) pathweight = 1;
		else if (dirjudge == 1) pathweight = 3;
		else  pathweight = 2;
		if (selectednode -> priority + pathweight < leftnode -> priority){
			leftnode -> priority = selectednode -> priority + pathweight;
			leftnode -> parent = selectednode;
		}
		//down
		if ((selectednode -> wall & 4) != 0) pathweight = INF_MAX;
		else if (dirjudge == 0 || dirjudge == -8) pathweight = 1;
		else if (dirjudge == 8) pathweight = 3;
		else  pathweight = 2;
		if (selectednode -> priority + pathweight < downnode -> priority){
			downnode -> priority = selectednode -> priority + pathweight;
			downnode -> parent = selectednode;
		}
		//up
		if ((selectednode -> wall & 1) != 0) pathweight = INF_MAX;
		else if (dirjudge == 0 || dirjudge == 8) pathweight = 1;
		else if (dirjudge == -8) pathweight = 3;
		else  pathweight = 2;
		if (selectednode -> priority + pathweight < upnode -> priority){
			upnode -> priority = selectednode -> priority + pathweight;
			upnode -> parent = selectednode;
		}
		//判定死循环
		old_snode = selectednode;
	}

//决策结果
//返回第一步
	//struct mazenode* node = &maze[aimy][aimx];
	//struct mazenode* startnode = &maze[posy][posx];
	while (1){
		//选出未访问节点中的最高优先级加入
		int shortest = INF_MAX;
		for (int i = 1; i < 7; i++){
			for (int j = 1; j < 7; j++){
				if (maze[i][j].visited == UNVISITED && maze[i][j].priority < shortest){
					shortest = maze[i][j].priority;
					selectednode = &maze[i][j];
				}
			}
		}
		if (old_snode == selectednode) return 255;	//死循环return
		selectednode -> visited = VISITED;
		//当目标点被选中时循环终止
		if (maze[aimy][aimx].visited == VISITED) break;
		//对此节点的邻居优先级做更新
		struct mazenode* rightnode = (selectednode + 1);
		struct mazenode* leftnode  = (selectednode - 1);
		struct mazenode* downnode  = (selectednode - 8);
		struct mazenode* upnode    = (selectednode + 8);
		//dirjudge
		int selectindex = selectednode - &maze[0][0];
		int parentindex = (selectednode -> parent) ? selectednode -> parent - &maze[0][0] : selectindex;
		int dirjudge = selectindex - parentindex;
		int pathweight = INF_MAX;
		//right
		if ((selectednode -> wall & 8) != 0) pathweight = INF_MAX;
		else if (dirjudge == 0 || dirjudge == 1) pathweight = 1;
		else if (dirjudge == -1) pathweight = 3;
		else  pathweight = 2;
		if (selectednode -> priority + pathweight < rightnode -> priority){
			rightnode -> priority = selectednode -> priority + pathweight;
			rightnode -> parent = selectednode;
		}
		//left
		if ((selectednode -> wall & 2) != 0) pathweight = INF_MAX;
		else if (dirjudge == 0 || dirjudge == -1) pathweight = 1;
		else if (dirjudge == 1) pathweight = 3;
		else  pathweight = 2;
		if (selectednode -> priority + pathweight < leftnode -> priority){
			leftnode -> priority = selectednode -> priority + pathweight;
			leftnode -> parent = selectednode;
		}
		//down
		if ((selectednode -> wall & 4) != 0) pathweight = INF_MAX;
		else if (dirjudge == 0 || dirjudge == -8) pathweight = 1;
		else if (dirjudge == 8) pathweight = 3;
		else  pathweight = 2;
		if (selectednode -> priority + pathweight < downnode -> priority){
			downnode -> priority = selectednode -> priority + pathweight;
			downnode -> parent = selectednode;
		}
		//up
		if ((selectednode -> wall & 1) != 0) pathweight = INF_MAX;
		else if (dirjudge == 0 || dirjudge == 8) pathweight = 1;
		else if (dirjudge == -8) pathweight = 3;
		else  pathweight = 2;
		if (selectednode -> priority + pathweight < upnode -> priority){
			upnode -> priority = selectednode -> priority + pathweight;
			upnode -> parent = selectednode;
		}
		//startpoint
		if (dirjudge == 0){
			switch(currentDir){
			case UP:
				downnode -> priority += 2;
				rightnode -> priority += 1;
				leftnode -> priority += 1;
				break;
			case DOWN:
				upnode -> priority += 2;
				rightnode -> priority += 1;
				leftnode -> priority += 1;
				break;
			case LEFT:
				rightnode -> priority += 2;
				upnode -> priority += 1;
				downnode -> priority += 1;
				break;
			case RIGHT:
				leftnode -> priority += 2;
				upnode -> priority += 1;
				downnode -> priority += 1;
				break;
			default: break;
			}
		}
		//判定死循环
		old_snode = selectednode;
	}
	return decision;
}

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

unsigned char PersonFinding (char currentDir, char posx, char posy, char p1bx, char p1by, char p2bx, char p2by, char wallrefresh){
	unsigned char decision = 255;

	return decision;
}
