#ifndef _MAZE_PATHFINDING_H_
#define _MAZE_PATHFINDING_H_

//添加障碍（节点坐标位置，选边：上0/左1/下2/右3），相邻节点间的围墙不需要重复添加
extern void AddWall (char x, char y, char side);

//将墙壁信息全部清空（含添加围墙和中间分界线）
extern void ClearWalls (void);

/*
路径规划函数
参数：当前方向（可以直接给UP，LEFT，DOWN，RIGHT，和main中定义相同），
      xy坐标，目标位置xy坐标（范围1-6,要在原坐标基础上加1），
      当前位置障碍情况（0-2位分别是右，前，左侧有无障碍）
返回值：ARRIVED = 0, MOVEFORE = 1, TURNLEFT = 2, TURNBACK = 3, TURNRIGHT = 4，fail = -1
*/
extern unsigned char PathFinding(char currentDir, char posx, char posy, char aimx, char aimy, char wallrefresh);
extern unsigned char PersonFinding (char currentDir, char posx, char posy, char p1bx, char p1by, char p2bx, char p2by, char wallrefresh, char* straightforward);

#endif
