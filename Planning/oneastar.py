def astar():
    import cv2
    import numpy as np
    import time
    import VisionCaptureApi
    import PX4MavCtrlV4 as PX4MavCtrl
    import math
    import threading
    import copy

    img = cv2.imread("competition.pgm")
    # 初始化，默认像素值为255表示可通行，否则不可通行
    img[img < 100] = 0
    img[img > 150] = 255
    # cv2.imshow("raw", img)
    # cv2.waitKey(0)

    # 在实际应用中，因为路径规划需要考虑载体运动学模型，把载体当做质点来考虑规划出来的轨迹往往需要做进一步的工作(如根据运动学约束,轨迹平滑，插值等)
    # 如果不考虑运动学模型，可行情况下对障碍物做膨胀处理，这样轨迹不会贴着实际障碍物
    grid = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grid[grid <= 200] = 1  # 1:表示不可通行
    grid[grid > 200] = 0  # 0:表示可通行

    # for col in range(len(grid[0])):
    #     for row in range(len(grid)):
    #         if(grid[row, col] == 1):
    #             if(row > 6 and row < len(grid[0] < 6) and col > 6 and col < len(grid) < 6):
    #                 grid[row-3:row+3, col-3:col+3] = 1

    grid = grid.tolist()
    origin_p = [70, 180]  # 设置起点
    goal_p = [38, 760]  # 设置目标点


    def AStar(grid: list, begin_p: list, goal_p: list, cost=2):
        assert ((grid[begin_p[0]][begin_p[1]] != 1)
                and (grid[goal_p[0]][goal_p[1]] != 1))
        # the cost map which pushes the path closer to the goal
        heuristic = [[0 for row in range(len(grid[0]))]
                    for col in range(len(grid))]
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                heuristic[i][j] = abs(i - goal_p[0]) + \
                    abs(j - goal_p[1])
                if grid[i][j] == 1:
                    # 在启发式地图里增加额外惩罚
                    heuristic[i][j] = 100000

        # 遍历方向
        delta = [[-1, -1],
                [-1, 0],
                [-1, 1],
                [0, -1],
                [0, 1],
                [1, -1],
                [1, 0],
                [1, 1]]

        close_matrix = [[0 for col in range(len(grid[0]))] for row in range(
            len(grid))]  # 用于存已经走过了路
        close_matrix[begin_p[0]][begin_p[1]] = 1
        action_matrix = [[0 for col in range(len(grid[0]))]
                        for row in range(len(grid))]  # 存储可能的路

        x = begin_p[0]
        y = begin_p[1]
        g = 0
        f = g + heuristic[begin_p[0]][begin_p[0]]
        cell = [[f, g, x, y]]

        found = False
        resign = False

        while not found and not resign:
            if len(cell) == 0:
                resign = True
                return None, None
            else:
                cell.sort()
                cell.reverse()
                next = cell.pop()
                x = next[2]
                y = next[3]
                g = next[1]
                f = next[0]

                if x == goal_p[0] and y == goal_p[1]:
                    found = True
                else:
                    for i in range(len(delta)):
                        x2 = x + delta[i][0]
                        y2 = y + delta[i][1]
                        # 判断可否通过那个点
                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                            if close_matrix[x2][y2] == 0 and grid[x2][y2] == 0:
                                g2 = g + cost
                                f2 = g2 + heuristic[x2][y2]
                                cell.append([f2, g2, x2, y2])
                                close_matrix[x2][y2] = 1
                                action_matrix[x2][y2] = i
        invpath = []
        x = goal_p[0]
        y = goal_p[1]
        invpath.append([x, y])  # 路径从起点到终点，因此需要反转
        while x != begin_p[0] or y != begin_p[1]:
            x2 = x - delta[action_matrix[x][y]][0]
            y2 = y - delta[action_matrix[x][y]][1]
            x = x2
            y = y2
            invpath.append([x, y])

        path = []
        for i in range(len(invpath)):
            path.append(invpath[len(invpath) - 1 - i])
        return path, action_matrix


    # print(img.shape)
    # # rgb_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # cv2.circle(img, (origin_p[1], origin_p[0]), 4, (0, 0, 255), 8)
    # cv2.circle(img, (goal_p[1], goal_p[0]), 4, (255, 0, 0), 8)

    path, a = AStar(grid, origin_p, goal_p)

    # for i in range(len(path)):
    #     cv2.circle(img, (path[i][1], path[i][0]), 1, (0, 255, 0), 1)

    # cv2.imshow("path", img)
    # cv2.waitKey(0)

    # path = [int(x) for x in path]

    # path=tuple(path)
    
    x=int(path[0][1])
    y=int(path[0][0])
    return 100
# num = astar()
# print(num)