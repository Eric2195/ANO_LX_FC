#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Ano_Math.h"
#include "ANO_LX.h"
#include "LX_FC_State.h"

//********************************pid参数**************************************//

/*水平位置1的pid参数*/
//y方向P 0.40 D 0.05
#define KP1 0.40f
#define KD1 0.05f

//x方向P 0.35 D 0.08
#define KP2 0.35f
#define KD2 0.08f

float y_move_pid(s16 cy)
{
	static float err_old;
	float err_d;
	float pid_out_y;
		err_d = cy - err_old;
		err_old = cy;
		pid_out_y = KP1*cy + KD1*err_d;
		pid_out_y = -LIMIT(pid_out_y,-10,10);
	return pid_out_y;
}

float x_move_pid(s16 cx)
{
	static float err_old;
	float err_d;
	float pid_out_x;
		err_d = cx - err_old;
		err_old = cx;
		pid_out_x = KP2*cx + KD2*err_d;
		pid_out_x = -LIMIT(pid_out_x,-10,10);
	return pid_out_x;
}

//*****************************************************************************//

//********************************路径规划部分**************************************//

// 由于当前项目没有 limits.h，手动定义 INT_MAX
#ifndef INT_MAX
#define INT_MAX 32767
#endif

typedef struct {
    Point data[MAX_CELLS];
    int front, rear;
} Queue;

int grid[ROWS][COLS];

Point barriers[BARRIER_COUNT];
Point accessible_cells[MAX_CELLS];
int accessible_count = 0;
Point final_path[MAX_PATH_LENGTH];
int final_path_length = 0;

int directions[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};

PathPoint path_points[MAX_PATH_POINTS];
uint8_t path_len_routine = 0;

// 简化测试路径：向 Y 方向走一格（50cm），然后返回原点
// 如需测试 X 方向，改为 {50, 0}
static const PathPoint test_path[] = {
    {0, 50},
    {0, 0}
};
#define TEST_PATH_LEN 2

s16 now_x = 0;
s16 now_y = 0;

void init_grid() {
    int i, j;
    for (i = 0; i < ROWS; i++)
        for (j = 0; j < COLS; j++)
            grid[i][j] = 1;
}

void init_queue(Queue *q) {
    q->front = q->rear = 0;
}

int is_queue_empty(Queue *q) {
    return q->front == q->rear;
}

void enqueue(Queue *q, Point p) {
    q->data[q->rear++] = p;
}

Point dequeue(Queue *q) {
    return q->data[q->front++];
}

int is_valid(int row, int col) {
    return row >= 0 && row < ROWS && col >= 0 && col < COLS;
}

int check_connectivity() {
    int visited[ROWS][COLS] = {0};
    Queue q;
    init_queue(&q);
    Point start = {0, 0};
    enqueue(&q, start);
    visited[0][0] = 1;
    int count = 1;

    while (!is_queue_empty(&q)) {
        Point cur = dequeue(&q);
        int i;
        for (i = 0; i < 4; i++) {
            int nr = cur.row + directions[i][0];
            int nc = cur.col + directions[i][1];
            if (is_valid(nr, nc) && !visited[nr][nc] && grid[nr][nc]) {
                visited[nr][nc] = 1;
                enqueue(&q, (Point){nr, nc});
                count++;
            }
        }
    }

    return count >= (ROWS * COLS - BARRIER_COUNT) * 4 / 5;
}

void generate_barriers() {
    int i;
    init_grid();
    for (i = 0; i < BARRIER_COUNT; i++) {
        if (is_valid(barriers[i].row, barriers[i].col))
            grid[barriers[i].row][barriers[i].col] = 0;
    }
}

void collect_accessible_cells() {
    int i, j;
    accessible_count = 0;
    for (i = 0; i < ROWS; i++)
        for (j = 0; j < COLS; j++)
            if (grid[i][j] == 1)
                accessible_cells[accessible_count++] = (Point){i, j};
}

int manhattan_distance(Point a, Point b) {
    int dr = a.row - b.row;
    int dc = a.col - b.col;
    return (dr >= 0 ? dr : -dr) + (dc >= 0 ? dc : -dc);
}

int find_shortest_path(Point start, Point end, Point path[], int max_len) {
    int visited[ROWS][COLS] = {0};
    int parent[ROWS][COLS][2];
    Queue q;
    init_queue(&q);
    enqueue(&q, start);
    visited[start.row][start.col] = 1;
    parent[start.row][start.col][0] = -1;
    parent[start.row][start.col][1] = -1;

    while (!is_queue_empty(&q)) {
        Point cur = dequeue(&q);
        if (cur.row == end.row && cur.col == end.col) {
            int len = 0;
            Point t = end;
            while (t.row != -1) {
                path[len++] = t;
                int pr = parent[t.row][t.col][0];
                int pc = parent[t.row][t.col][1];
                t.row = pr; t.col = pc;
            }
            {
                int i;
                for (i = 0; i < len / 2; i++) {
                    Point tmp = path[i];
                    path[i] = path[len - 1 - i];
                    path[len - 1 - i] = tmp;
                }
            }
            return len;
        }
        {
            int i;
            for (i = 0; i < 4; i++) {
                int nr = cur.row + directions[i][0];
                int nc = cur.col + directions[i][1];
                if (is_valid(nr, nc) && !visited[nr][nc] && grid[nr][nc]) {
                    visited[nr][nc] = 1;
                    parent[nr][nc][0] = cur.row;
                    parent[nr][nc][1] = cur.col;
                    enqueue(&q, (Point){nr, nc});
                }
            }
        }
    }

    return 0;
}

void nearest_neighbor_tsp(Point order[]) {
    int visited[MAX_CELLS] = {0};
    Point cur = {0, 0};
    int count = 0;

    int start_idx = -1;
    int i;
    for (i = 0; i < accessible_count; i++)
        if (accessible_cells[i].row == 0 && accessible_cells[i].col == 0)
            { start_idx = i; break; }

    if (start_idx != -1) {
        order[count++] = cur;
        visited[start_idx] = 1;
    }

    while (count < accessible_count) {
        int nearest_idx = -1, min_dist = INT_MAX;
        for (i = 0; i < accessible_count; i++) {
            if (!visited[i]) {
                int dist = manhattan_distance(cur, accessible_cells[i]);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_idx = i;
                }
            }
        }
        if (nearest_idx == -1) break;
        cur = accessible_cells[nearest_idx];
        order[count++] = cur;
        visited[nearest_idx] = 1;
    }
}

void build_full_path(Point order[], int count) {
    int i;
    final_path_length = 0;
    for (i = 0; i < count; i++) {
        Point start = order[i];
        Point end = order[(i + 1) % count];
        Point seg[MAX_PATH_LENGTH];
        int seg_len = find_shortest_path(start, end, seg, MAX_PATH_LENGTH);
        int j;
        for (j = (i == 0 ? 0 : 1); j < seg_len; j++) {
            final_path[final_path_length++] = seg[j];
        }
    }
}

void real_routine()
{
    int i;
    path_len_routine = 0;
    for (i = 0; i < final_path_length; i++) {
        path_points[path_len_routine].x = GRID_SIZE_CM * final_path[i].row;
        path_points[path_len_routine].y = GRID_SIZE_CM * final_path[i].col;
        path_len_routine++;
    }
}

void run_path_planner(void) {
    generate_barriers();
    collect_accessible_cells();
    Point visit_order[MAX_CELLS];
    nearest_neighbor_tsp(visit_order);
    build_full_path(visit_order, accessible_count);
    real_routine();
}

//**********************************************************************************//

void UserTask_OneKeyCmd(void)
{
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step = 0;
    static u16 delay_cnt_ms = 0;
    static u16 hover_delay_ms = 0;
    static u8 current_path_index = 0;

    if (rc_in.fail_safe == 0)
    {
        // CH6 低位：一键降落
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            if (one_key_land_f == 0)
            {
                one_key_land_f = OneKey_Land();
            }
        }
        else
        {
            one_key_land_f = 0;
        }

        // CH6 高位：执行任务
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            if (one_key_mission_f == 0)
            {
                one_key_mission_f = 1;
                mission_step = 1;
                current_path_index = 0;
                delay_cnt_ms = 0;
                hover_delay_ms = 0;
            }
        }
        else
        {
            one_key_mission_f = 0;
        }

        if (one_key_mission_f == 1)
        {
            switch(mission_step)
            {
                case 0:
                {
                    delay_cnt_ms = 0;
                    hover_delay_ms = 0;
                }
                break;

                // 切换程控模式
                case 1:
                {
                    mission_step += LX_Change_Mode(2);
                }
                break;

                // 解锁
                case 2:
                {
                    mission_step += FC_Unlock();
                }
                break;

                // 延时2s
                case 3:
                {
                    delay_cnt_ms += 20;
                    if (delay_cnt_ms >= 2000)
                    {
                        delay_cnt_ms = 0;
                        mission_step++;
                    }
                }
                break;

                // 起飞到50cm
                case 4:
                {
                    mission_step += OneKey_Takeoff(50);
                }
                break;

                // 悬停稳定3s
                case 5:
                {
                    delay_cnt_ms += 20;
                    if (delay_cnt_ms >= 3000)
                    {
                        delay_cnt_ms = 0;
                        mission_step++;
                    }
                }
                break;

                // 航点跟踪（简化测试：走固定测试路径）
                case 6:
                {
                    if (current_path_index < TEST_PATH_LEN)
                    {
                        s16 target_x = test_path[current_path_index].x;
                        s16 target_y = test_path[current_path_index].y;

                        s16 dx_u = target_x - now_x;
                        s16 dy_u = target_y - now_y;

                        // X方向：大偏差快速接近，小偏差PID，死区停止
                        if (ABS(dx_u) > 10)
                            rt_tar.st_data.vel_x = (dx_u > 0) ? -10 : 10;
                        else if (ABS(dx_u) > 2)
                            rt_tar.st_data.vel_x = x_move_pid(dx_u);
                        else
                            rt_tar.st_data.vel_x = 0;

                        // Y方向：同上
                        if (ABS(dy_u) > 10)
                            rt_tar.st_data.vel_y = (dy_u > 0) ? -10 : 10;
                        else if (ABS(dy_u) > 2)
                            rt_tar.st_data.vel_y = y_move_pid(dy_u);
                        else
                            rt_tar.st_data.vel_y = 0;

                        rt_tar.st_data.vel_z = 0;

                        if (ABS(dx_u) < 15 && ABS(dy_u) < 15)
                        {
                            // 到达，悬停500ms
                            rt_tar.st_data.vel_x = 0;
                            rt_tar.st_data.vel_y = 0;
                            hover_delay_ms += 20;
                            if (hover_delay_ms >= 500)
                            {
                                hover_delay_ms = 0;
                                current_path_index++;
                            }
                        }
                    }
                    else
                    {
                        // 所有航点完成
                        rt_tar.st_data.vel_x = 0;
                        rt_tar.st_data.vel_y = 0;
                        rt_tar.st_data.vel_z = 0;
                        mission_step++;
                    }
                }
                break;

                // 降落
                case 7:
                {
                    mission_step += OneKey_Land();
                }
                break;

                default:
                    break;
            }
        }
        else
        {
            // 紧急退出任务：立即清零速度输出，防止残留指令干扰降落
            rt_tar.st_data.vel_x = 0;
            rt_tar.st_data.vel_y = 0;
            rt_tar.st_data.vel_z = 0;

            mission_step = 0;
            delay_cnt_ms = 0;
            hover_delay_ms = 0;
            current_path_index = 0;
        }
    }
}
