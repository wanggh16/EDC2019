//#include <stdio.h>
#include "maze_pathfinding.h"

#define STABLE 0
#define INF_MAX 1000
#define UNVISITED 0
#define VISITED 1
#define BALLMODE 1
#define PERSONMODE 0

#define ARRIVED 0
#define TURNLEFT 14
#define TURNRIGHT 15
#define TURNBACK 16
#define MOVEFORE 11

#define DIRECTLEFT 17
#define DIRECTRIGHT 18
#define DIRECTBACK 19


enum DIRECTION{//maze里的绝对方向
	UP = 1, LEFT, DOWN, RIGHT
};

struct mazenode{
	char wall;	//0-3位分别是上，左，下，右侧有无障碍，高位=0

	//路径规划中使用的变量
	struct mazenode* parent;
	int priority;
	char visited;
	char wallcomplete;
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
			maze[i][j].wallcomplete = 0;
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

void ReturnFirstStep(char currentDir, char posx, char posy, char aimx, char aimy, unsigned char* dec);
void MakeDecision(char currentDir, char posx, char posy, char aimx, char aimy, unsigned char* dec, char* straightforward);

//参数：当前方向，xy坐标，目标位置xy坐标（范围1-6,要在原坐标基础上加1），当前位置障碍情况（0-2位分别是后，右，前，左侧有无障碍）
unsigned char MakePath (char currentDir, char posx, char posy, char aimx, char aimy, char wallrefresh, int* pri_total, char mode,char* straightforward){
	unsigned char decision = 255;

	//起点终点重合返回ARRIVED = 0
	if (posx == aimx && posy == aimy) return ARRIVED;

	//更新墙壁信息
	switch(currentDir){
	case RIGHT:
		if (wallrefresh & 8) AddWall(posx,posy,LEFT);
		if (wallrefresh & 4) AddWall(posx,posy,UP);
		if (wallrefresh & 2) AddWall(posx,posy,RIGHT);
		if (wallrefresh & 1) AddWall(posx,posy,DOWN);
		break;
	case UP:
		if (wallrefresh & 8) AddWall(posx,posy,DOWN);
		if (wallrefresh & 4) AddWall(posx,posy,LEFT);
		if (wallrefresh & 2) AddWall(posx,posy,UP);
		if (wallrefresh & 1) AddWall(posx,posy,RIGHT);
		break;
	case DOWN:
		if (wallrefresh & 8) AddWall(posx,posy,UP);
		if (wallrefresh & 4) AddWall(posx,posy,RIGHT);
		if (wallrefresh & 2) AddWall(posx,posy,DOWN);
		if (wallrefresh & 1) AddWall(posx,posy,LEFT);
		break;
	case LEFT:
		if (wallrefresh & 8) AddWall(posx,posy,RIGHT);
		if (wallrefresh & 4) AddWall(posx,posy,DOWN);
		if (wallrefresh & 2) AddWall(posx,posy,LEFT);
		if (wallrefresh & 1) AddWall(posx,posy,UP);
		break;
	default: break;
	}
	maze[posy][posx].wallcomplete = 1;

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
		//startpoint
		if (dirjudge == 0){
			switch(currentDir){
			case UP:
				downnode -> priority += 1;//2;
				rightnode -> priority += 1;
				leftnode -> priority += 1;
				break;
			case DOWN:
				upnode -> priority += 1;//2;
				rightnode -> priority += 1;
				leftnode -> priority += 1;
				break;
			case LEFT:
				rightnode -> priority += 1;//2;
				upnode -> priority += 1;
				downnode -> priority += 1;
				break;
			case RIGHT:
				leftnode -> priority += 1;//2;
				upnode -> priority += 1;
				downnode -> priority += 1;
				break;
			default: break;
			}
		}
		//判定死循环
		old_snode = selectednode;
	}

	//决策结果
	/*path test print
	int path[6][6] = {0};
	for (mazenode* node = &maze[aimy][aimx]; node != NULL; node = node->parent){
	int x = (node - &maze[0][0]) / 8 - 1;
	int y = (node - &maze[0][0]) % 8 - 1;
	path[x][y] = node->priority;
	}
	printf("path found:\n");
	for (int i = 5; i >= 0; i--){
	for (int j = 0; j < 6; j++){
	printf("%d ",path[i][j]);
	}
	printf("\n");
	}
	printf("################################\n");*/

	//返回第一步
	if (mode == BALLMODE){
		ReturnFirstStep(currentDir,posx,posy,aimx,aimy,&decision);
	}
	else if (mode == PERSONMODE){
		MakeDecision(currentDir,posx,posy,aimx,aimy,&decision,straightforward);
	}
	else return 255;

	*pri_total = maze[aimy][aimx].priority;

	//printf("decision: %d\n",(int)decision);
	return decision;
}

/*********************************************************************************/
/*********************************************************************************/

//不带平移时返回第一步
void ReturnFirstStep(char currentDir, char posx, char posy, char aimx, char aimy, unsigned char* dec){
	unsigned char decision;
	//返回第一步
	struct mazenode* node = &maze[aimy][aimx];
	struct mazenode* startnode = &maze[posy][posx];
	while (1){
		if (node ->parent == 0){
			decision = 255;
			return;
		}
		if (node -> parent == startnode){
			char x = (node - &maze[0][0]) % 8;
			char y = (node - &maze[0][0]) / 8;
			if (x - posx == 1){//right
				switch (currentDir){
				case UP:	decision = TURNRIGHT; break;
				case LEFT:	decision = TURNBACK;  break;
				case DOWN:	decision = TURNLEFT;  break;
				case RIGHT:	decision = MOVEFORE;  break;
				default: break;
				}
			}
			else if (x - posx == -1){//left
				switch (currentDir){
				case UP:	decision = TURNLEFT;  break;
				case LEFT:	decision = MOVEFORE;  break;
				case DOWN:	decision = TURNRIGHT; break;
				case RIGHT:	decision = TURNBACK;  break;
				default: break;
				}
			}
			else if (y - posy == 1){//up
				switch (currentDir){
				case UP:	decision = MOVEFORE;  break;
				case LEFT:	decision = TURNRIGHT; break;
				case DOWN:	decision = TURNBACK;  break;
				case RIGHT:	decision = TURNLEFT;  break;
				default: break;
				}
			}
			else if (y - posy == -1){//down
				switch (currentDir){
				case UP:	decision = TURNBACK;  break;
				case LEFT:	decision = TURNLEFT;  break;
				case DOWN:	decision = MOVEFORE;  break;
				case RIGHT:	decision = TURNRIGHT; break;
				default: break;
				}
			}
			break;
		}
		node = node->parent;
	}
	*dec = decision;
}

//决定平移转向
void MakeDecision (char currentDir, char posx, char posy, char aimx, char aimy, unsigned char* dec, char* straightforward){
	unsigned char decision;
	//还原路径
	char pathx[21] = {0};
	char pathy[21] = {0};
	int total_point = 0;	//起点到终点的路径中包含的节点个数
	for (struct mazenode* node = &maze[aimy][aimx]; node != 0; node = node->parent){
		pathy[total_point] = (node - &maze[0][0]) / 8;
		pathx[total_point] = (node - &maze[0][0]) % 8;
		total_point++;
	}

	char goLEFT = TURNLEFT;
	char goRIGHT = TURNRIGHT;
	char goBACK = TURNBACK;

	//找到下一个拐点
	int turnindex = 0;
	for (int i = total_point - 1; i >= 0; i--){
		if (pathx[i - 2] - pathx[i - 1] != pathx[i - 1] - pathx[i] 
		|| pathy[i - 2] - pathy[i - 1] != pathy[i - 1] - pathy[i]){
			turnindex = i - 1;
			break;
		}
	}
	//当路径上的下一个点墙壁信息已知且不是拐点
	if (turnindex != total_point - 2 
		&& maze[pathy[total_point - 2]][pathx[total_point - 2]].wallcomplete == 1){
		*straightforward = 1;
	}
	else{
		*straightforward = 0;
	}

	if (total_point <= 3){//目的地与起点之间距离<=2时直接平移
		goBACK = DIRECTBACK;
		goLEFT = DIRECTLEFT;
		goRIGHT = DIRECTRIGHT;
	}
	else{
		//两格内转向且下一次转向方向与现在相同时才平移
		if (total_point - turnindex <= 4) goBACK = DIRECTBACK;
		if (total_point - turnindex <= 3){
			//goBACK = DIRECTBACK;
			//char nextDir = 0;
			//if (pathx[turnindex - 1] - pathx[turnindex] == 1) nextDir = RIGHT;
			//else if (pathx[turnindex - 1] - pathx[turnindex] == -1) nextDir = LEFT;
			//else if (pathy[turnindex - 1] - pathy[turnindex] == 1) nextDir = UP;
			//else if (pathy[turnindex - 1] - pathy[turnindex] == -1) nextDir = DOWN;

			//if (nextDir == currentDir){
			goLEFT = DIRECTLEFT;
			goRIGHT = DIRECTRIGHT;
			//}
		}
	}

#if STABLE == 1
	goLEFT = TURNLEFT;
	goRIGHT = TURNRIGHT;
	goBACK = TURNBACK;
#endif

	if (pathx[total_point - 2] - pathx[total_point - 1] == 1){//right
		switch (currentDir){
		case UP:	decision = goRIGHT;		break;
		case LEFT:	decision = goBACK;		break;
		case DOWN:	decision = goLEFT;		break;
		case RIGHT:	decision = MOVEFORE;	break;
		default: break;
		}
	}
	else if (pathx[total_point - 2] - pathx[total_point - 1] == -1){//left
		switch (currentDir){
		case UP:	decision = goLEFT;		break;
		case LEFT:	decision = MOVEFORE;	break;
		case DOWN:	decision = goRIGHT; 	break;
		case RIGHT:	decision = goBACK;  	break;
		default: break;
		}
	}
	else if (pathy[total_point - 2] - pathy[total_point - 1] == 1){//up
		switch (currentDir){
		case UP:	decision = MOVEFORE;	break;
		case LEFT:	decision = goRIGHT;		break;
		case DOWN:	decision = goBACK;		break;
		case RIGHT:	decision = goLEFT;  	break;
		default: break;
		}
	}
	else if (pathy[total_point - 2] - pathy[total_point - 1] == -1){//down
		switch (currentDir){
		case UP:	decision = goBACK;  	break;
		case LEFT:	decision = goLEFT;  	break;
		case DOWN:	decision = MOVEFORE;	break;
		case RIGHT:	decision = goRIGHT; 	break;
		default: break;
		}
	}
	*dec = decision;
}

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

unsigned char PathFinding(char currentDir, char posx, char posy, char aimx, char aimy, char wallrefresh){
	int pri = 0;
	char useless;
	unsigned char decision = MakePath(currentDir,posx,posy,aimx,aimy,wallrefresh,&pri,BALLMODE,&useless);
	return decision;
}

unsigned char PersonFinding (char currentDir, char posx, char posy, char p1bx, char p1by, char p2bx, char p2by, char wallrefresh, char* straightforward){
	unsigned char decision1 = 255, decision2 = 255;
	char straight1 = 0, straight2 = 0;
	int pri_1 = INF_MAX, pri_2 = INF_MAX;
	decision1 = MakePath(currentDir,posx,posy,p1bx,p1by,wallrefresh,&pri_1,PERSONMODE,&straight1);
	decision2 = MakePath(currentDir,posx,posy,p2bx,p2by,wallrefresh,&pri_2,PERSONMODE,&straight2);

	//printf("****straightforward: %d\n",(int)straight1);
	/*printf("pri compare:%d %d ——",pri_1,pri_2);
	if (pri_1 < pri_2) printf("1\n");
	else printf("2\n");*/

	if (pri_1 < pri_2){
		*straightforward = straight1;
		return decision1;
	}
	else{
		*straightforward = straight2;
		return decision2;
	}
}

/*int main(void){
	ClearWalls();

	//AddWall(1,3,UP);
	//PathFinding(RIGHT,1,5,5,1,0);
	char straight = 0;
	PersonFinding(DOWN,1,2,1,3,1,3,0,&straight);
	PersonFinding(DOWN,1,3,1,1,1,1,0,&straight);

	//print wall condition
	printf("wall condition:\n");
	for (int i = 7; i >= 0; i--){
		for (int j = 0; j < 8; j++){
			printf("%d\t",(int)maze[i][j].wall);
		}
		printf("\n");
	}
	return 0;
}*/
