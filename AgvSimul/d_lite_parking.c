/**
* @file d_lite_parking.c
* @brief D* Lite 알고리즘을 이용한 다중 에이전트 주차 시뮬레이션
* @details 로봇이 움직이면서 경로를 점진적으로 수정하는 D* Lite 알고리즘을 적용했습니다.
*/
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <stdarg.h>
#include <ctype.h>

#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#include <termios.h>
#define SLEEP(ms) usleep(ms * 1000)
#endif

// =============================================================================
// --- 1. 상수 및 전역 변수 정의 ---
// =============================================================================
#define TRUE 1
#define FALSE 0
#define INPUT_BUFFER_SIZE 100
#define DISPLAY_BUFFER_SIZE 16384

// --- ANSI 컬러 코드 ---
#define C_NRM "\x1B[0m"
#define C_RED "\x1B[31m"
#define C_GRN "\x1B[32m"
#define C_YEL "\x1B[33m"
#define C_BLU "\x1B[34m"
#define C_MAG "\x1B[35m"
#define C_CYN "\x1B[36m"
#define C_WHT "\x1B[37m"
#define C_GRY "\x1B[90m"
#define C_B_RED "\x1B[1;31m"
#define C_B_GRN "\x1B[1;32m"
#define C_B_YEL "\x1B[1;33m"
#define C_B_MAG "\x1B[1;35m"
#define C_B_CYN "\x1B[1;36m"
#define C_B_WHT "\x1B[1;37m"

// --- 시뮬레이션 그리드 및 에이전트 설정 ---
#define GRID_WIDTH 37
#define GRID_HEIGHT 9
#define MAX_AGENTS 3
#define MAX_GOALS (GRID_WIDTH * GRID_HEIGHT)
#define INF DBL_MAX
#define NUM_DIRECTIONS 4

// --- 시뮬레이션 설정 값 ---
#define DISTANCE_BEFORE_CHARGE 300.0
#define CHARGE_TIME 20
#define MAX_CHARGE_STATIONS 10
#define MAX_PHASES 20
#define REALTIME_MODE_TIMELIMIT 10000
#define MAX_TASKS 50
#define MAX_SPEED_MULTIPLIER 100.0f
#define EVENT_GENERATION_INTERVAL 5

// --- 로그 및 UI 설정 ---
#define LOG_BUFFER_LINES 5
#define LOG_BUFFER_WIDTH 256
#define STATUS_STRING_WIDTH 25

// =============================================================================
// --- 2. 자료구조 정의 ---
// =============================================================================
typedef struct { double k1; double k2; } Key;

typedef struct Node {
    int x, y;
    double g, rhs;
    Key key;
    int is_obstacle;
    int is_goal;
    int is_temp;
    int is_parked;
    int reserved_by_agent;
    int in_pq;
    int pq_index;
} Node;

typedef enum {
    IDLE,
    GOING_TO_PARK,
    RETURNING_HOME_EMPTY,
    GOING_TO_COLLECT,
    RETURNING_WITH_CAR,
    GOING_TO_CHARGE,
    CHARGING,
    RETURNING_HOME_MAINTENANCE,
    AVOIDING_COLLISION
} AgentState;

// D* Lite를 위해 Pathfinder에 대한 전방 선언
struct Pathfinder;

/** @struct Agent
 * @brief D* Lite 적용을 위해 각 에이전트가 자신의 경로 탐색기(Pathfinder)를 소유합니다.
 */
typedef struct {
    int id;
    char symbol;
    Node* pos;
    Node* home_base;
    Node* goal;
    AgentState state;
    double total_distance_traveled;
    int charge_timer;
    struct Pathfinder* pf; // D* Lite를 위한 에이전트별 경로 탐색기
} Agent;

typedef struct {
    Node** nodes;
    int size;
    int capacity;
} PriorityQueue;

typedef enum { PARK_PHASE, EXIT_PHASE } PhaseType;

typedef struct {
    PhaseType type;
    int task_count;
    char type_name[10];
} DynamicPhase;

typedef enum { TASK_NONE, TASK_PARK, TASK_EXIT } TaskType;

typedef struct TaskNode {
    TaskType type;
    struct TaskNode* next;
} TaskNode;

typedef enum { MODE_UNINITIALIZED, MODE_CUSTOM, MODE_REALTIME } SimulationMode;

typedef struct {
    Node grid[GRID_HEIGHT][GRID_WIDTH];
    Node* goals[MAX_GOALS];
    int num_goals;
    Node* charge_stations[MAX_CHARGE_STATIONS];
    int num_charge_stations;
} GridMap;

typedef struct {
    Agent agents[MAX_AGENTS];
    int total_cars_parked;
} AgentManager;

/** @struct Pathfinder
 * @brief D* Lite 알고리즘의 상태를 저장하는 자료구조.
 */
typedef struct Pathfinder {
    PriorityQueue pq;
    Node* robot_pos;  // D* Lite: 계속 움직이는 로봇의 현재 위치
    Node* goal_node;  // 목표 지점
    double k_m;       // D* Lite: 로봇 이동에 따른 휴리스틱 보정값
} Pathfinder;

typedef struct {
    SimulationMode mode;
    int time_step;
    int simulation_speed;
    float speed_multiplier;
    DynamicPhase phases[MAX_PHASES];
    int num_phases;
    int current_phase_index;
    int tasks_completed_in_phase;
    TaskNode* task_queue_head;
    TaskNode* task_queue_tail;
    int task_count;
    int park_chance;
    int exit_chance;
} ScenarioManager;

typedef struct {
    char logs[LOG_BUFFER_LINES][LOG_BUFFER_WIDTH];
    int log_head;
    int log_count;
} Logger;

typedef struct {
    GridMap* map;
    AgentManager* agent_manager;
    ScenarioManager* scenario_manager;
    Logger* logger;
} Simulation;

// =============================================================================
// --- 3. 함수 원형 선언 ---
// =============================================================================
// --- 시뮬레이션 생성/소멸 함수 ---
Simulation* simulation_create();
void simulation_destroy(Simulation* sim);

// --- 시스템 및 UI 함수 ---
void system_enable_virtual_terminal();
void ui_clear_screen_optimized();

// --- 우선순위 큐 (Priority Queue) 함수 ---
void pq_init(PriorityQueue* pq, int capacity);
void pq_free(PriorityQueue* pq);
void pq_push(PriorityQueue* pq, Node* node);
Node* pq_pop(PriorityQueue* pq);
void pq_remove(PriorityQueue* pq, Node* node);
int pq_contains(const Node* node);
Key pq_top_key(const PriorityQueue* pq);

// --- 그리드 (GridMap) 관리 함수 ---
int grid_is_valid_coord(int x, int y);
int grid_is_node_blocked(const GridMap* map, const AgentManager* agent_manager, const Node* n);

// --- 경로 탐색 (Pathfinder) 함수 ---
Pathfinder* pathfinder_create(Node* robot_pos, Node* goal);
void pathfinder_destroy(Pathfinder* pf);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, AgentManager* agent_manager);
Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager);

// --- 에이전트 (AgentManager) 관리 함수 ---
void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]);
void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger);
void agent_manager_update_charge_state(AgentManager* manager, Logger* logger);
void agent_manager_assign_tasks_to_agents(AgentManager* manager, ScenarioManager* scenario, GridMap* map, Logger* logger);

// --- 시뮬레이션 (Simulation) 관리 함수 ---
void simulation_run(Simulation* sim);
int simulation_setup(Simulation* sim);
void logger_log(Logger* logger, const char* format, ...);

// =============================================================================
// --- 4. 시스템 및 UI 함수 구현 ---
// =============================================================================
void system_enable_virtual_terminal() {
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut == INVALID_HANDLE_VALUE) return;
    DWORD dwMode = 0;
    if (!GetConsoleMode(hOut, &dwMode)) return;
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hOut, dwMode);
#endif
}

void ui_clear_screen_optimized() {
#ifdef _WIN32
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    COORD coordScreen = { 0, 0 };
    SetConsoleCursorPosition(hConsole, coordScreen);
#else
    printf("\033[H");
#endif
}

// =============================================================================
// --- 5. 우선순위 큐 (Min-Heap) 구현 ---
// =============================================================================
static int compare_keys(Key k1, Key k2) {
    if (k1.k1 < k2.k1 - 1e-9) return -1;
    if (k1.k1 > k2.k1 + 1e-9) return 1;
    if (k1.k2 < k2.k2 - 1e-9) return -1;
    if (k1.k2 > k2.k2 + 1e-9) return 1;
    return 0;
}

void pq_init(PriorityQueue* pq, int capacity) {
    pq->nodes = (Node**)malloc(sizeof(Node*) * capacity);
    pq->size = 0;
    pq->capacity = capacity;
}

void pq_free(PriorityQueue* pq) {
    if (pq && pq->nodes) {
        free(pq->nodes);
        pq->nodes = NULL;
    }
}

static void swap_nodes(Node** a, Node** b) {
    Node* t = *a; *a = *b; *b = t;
    int temp_idx = (*a)->pq_index;
    (*a)->pq_index = (*b)->pq_index;
    (*b)->pq_index = temp_idx;
}

static void heapify_up(PriorityQueue* pq, int i) {
    if (i == 0) return;
    int p = (i - 1) / 2;
    if (compare_keys(pq->nodes[i]->key, pq->nodes[p]->key) < 0) {
        swap_nodes(&pq->nodes[i], &pq->nodes[p]);
        heapify_up(pq, p);
    }
}

static void heapify_down(PriorityQueue* pq, int i) {
    int l = 2 * i + 1, r = 2 * i + 2, s = i;
    if (l < pq->size && compare_keys(pq->nodes[l]->key, pq->nodes[s]->key) < 0) s = l;
    if (r < pq->size && compare_keys(pq->nodes[r]->key, pq->nodes[s]->key) < 0) s = r;
    if (s != i) {
        swap_nodes(&pq->nodes[i], &pq->nodes[s]);
        heapify_down(pq, s);
    }
}

void pq_push(PriorityQueue* pq, Node* n) {
    if (pq->size >= pq->capacity) return;
    n->in_pq = TRUE;
    n->pq_index = pq->size;
    pq->nodes[pq->size++] = n;
    heapify_up(pq, pq->size - 1);
}

Node* pq_pop(PriorityQueue* pq) {
    if (pq->size == 0) return NULL;
    Node* top = pq->nodes[0];
    top->in_pq = FALSE;
    top->pq_index = -1;
    pq->size--;
    if (pq->size > 0) {
        pq->nodes[0] = pq->nodes[pq->size];
        pq->nodes[0]->pq_index = 0;
        heapify_down(pq, 0);
    }
    return top;
}

void pq_remove(PriorityQueue* pq, Node* n) {
    if (!n->in_pq) return;
    int idx = n->pq_index;
    pq->size--;
    if (idx != pq->size) {
        pq->nodes[idx] = pq->nodes[pq->size];
        pq->nodes[idx]->pq_index = idx;
        if (idx > 0 && compare_keys(pq->nodes[idx]->key, pq->nodes[(idx - 1) / 2]->key) < 0) {
            heapify_up(pq, idx);
        }
        else {
            heapify_down(pq, idx);
        }
    }
    n->in_pq = FALSE;
    n->pq_index = -1;
}

int pq_contains(const Node* n) { return n->in_pq; }

Key pq_top_key(const PriorityQueue* pq) {
    if (pq->size == 0) return (Key) { INF, INF };
    return pq->nodes[0]->key;
}

// --- Logger Implementation ---
Logger* logger_create() {
    Logger* logger = (Logger*)calloc(1, sizeof(Logger));
    if (!logger) {
        perror("Logger allocation failed");
        exit(1);
    }
    return logger;
}

void logger_destroy(Logger* logger) {
    if (logger) free(logger);
}

void logger_log(Logger* logger, const char* format, ...) {
    va_list args;
    va_start(args, format);
    int current_log_index = (logger->log_head + logger->log_count) % LOG_BUFFER_LINES;
    vsnprintf(logger->logs[current_log_index], LOG_BUFFER_WIDTH, format, args);
    va_end(args);

    if (logger->log_count < LOG_BUFFER_LINES) {
        logger->log_count++;
    }
    else {
        logger->log_head = (logger->log_head + 1) % LOG_BUFFER_LINES;
    }
}

// --- GridMap Implementation ---
static void grid_map_load_from_embedded_map(GridMap* map, AgentManager* agent_manager) {
    const char* embedded_map_data =
        "1111111111111111111111111111111111111\n"
        "C01GGG1GG1GGG1GGG1GGG1GGG1GGG1G11G111\n"
        "A000000000000000000000000000000000001\n"
        "B0000000000000000000000000000000000001\n"
        "1111GG1GG1GGG10001GGG1GGG1GGG1100E111\n"
        "111111111111110001GGG1GGG1GGG11001111\n"
        "100000000000000000000000000000000E111\n"
        "100000000000000000000000000000000E111\n"
        "11111111111111GGG1GGG1GGG1GGG1GG11111\n";

    map->num_goals = 0;
    map->num_charge_stations = 0;
    int map_idx = 0;
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            char ch;
            do { ch = embedded_map_data[map_idx++]; } while (ch == '\n' || ch == '\r');
            if (ch == '\0') {
                fprintf(stderr, C_B_RED "Error: Embedded map data is shorter than expected.\n" C_NRM);
                exit(1);
            }

            Node* n = &map->grid[y][x];
            *n = (Node){ .x = x, .y = y, .g = INF, .rhs = INF, .is_obstacle = FALSE,
            .is_goal = FALSE, .is_temp = FALSE, .is_parked = FALSE,
            .reserved_by_agent = -1, .in_pq = FALSE, .pq_index = -1 };

            switch (ch) {
            case '1': n->is_obstacle = TRUE; break;
            case 'A': agent_manager->agents[0].pos = n; agent_manager->agents[0].home_base = n; break;
            case 'B': agent_manager->agents[1].pos = n; agent_manager->agents[1].home_base = n; break;
            case 'C': agent_manager->agents[2].pos = n; agent_manager->agents[2].home_base = n; break;
            case 'G': n->is_goal = TRUE; if (map->num_goals < MAX_GOALS) map->goals[map->num_goals++] = n; break;
            case 'E': if (map->num_charge_stations < MAX_CHARGE_STATIONS) map->charge_stations[map->num_charge_stations++] = n; break;
            }
        }
    }
    if (map->num_charge_stations == 0) {
        fprintf(stderr, C_B_RED "Error: No charge stations ('E') found in the map data.\n" C_NRM);
        exit(1);
    }
}

GridMap* grid_map_create(AgentManager* agent_manager) {
    GridMap* map = (GridMap*)calloc(1, sizeof(GridMap));
    if (!map) {
        perror("GridMap allocation failed");
        exit(1);
    }
    grid_map_load_from_embedded_map(map, agent_manager);
    return map;
}

void grid_map_destroy(GridMap* map) {
    if (map) free(map);
}

int grid_is_valid_coord(int x, int y) {
    return x >= 0 && x < GRID_WIDTH&& y >= 0 && y < GRID_HEIGHT;
}

int grid_is_node_blocked(const GridMap* map, const AgentManager* agent_manager, const Node* n) {
    if (n->is_obstacle || n->is_parked || n->is_temp) {
        return TRUE;
    }
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (agent_manager->agents[i].pos == n && agent_manager->agents[i].state == CHARGING) {
            return TRUE;
        }
    }
    return FALSE;
}

// --- AgentManager Implementation ---
AgentManager* agent_manager_create() {
    AgentManager* manager = (AgentManager*)calloc(1, sizeof(AgentManager));
    if (!manager) {
        perror("AgentManager allocation failed");
        exit(1);
    }
    for (int i = 0; i < MAX_AGENTS; i++) {
        manager->agents[i].id = i;
        manager->agents[i].symbol = 'A' + i;
        manager->agents[i].state = IDLE;
        manager->agents[i].pf = NULL; // 경로 탐색기 초기화
    }
    return manager;
}

void agent_manager_destroy(AgentManager* manager) {
    if (manager) {
        for (int i = 0; i < MAX_AGENTS; i++) {
            if (manager->agents[i].pf) {
                pathfinder_destroy(manager->agents[i].pf); // 에이전트의 경로 탐색기 메모리 해제
            }
        }
        free(manager);
    }
}

// --- ScenarioManager Implementation ---
static void scenario_manager_clear_task_queue(ScenarioManager* manager) {
    TaskNode* current = manager->task_queue_head;
    TaskNode* next_node;
    while (current != NULL) {
        next_node = current->next;
        free(current);
        current = next_node;
    }
    manager->task_queue_head = NULL;
    manager->task_queue_tail = NULL;
    manager->task_count = 0;
}

ScenarioManager* scenario_manager_create() {
    ScenarioManager* manager = (ScenarioManager*)calloc(1, sizeof(ScenarioManager));
    if (!manager) {
        perror("ScenarioManager allocation failed");
        exit(1);
    }
    manager->simulation_speed = 100;
    manager->speed_multiplier = 1.0f;
    manager->park_chance = 40;
    manager->exit_chance = 30;
    manager->task_queue_head = NULL;
    manager->task_queue_tail = NULL;
    manager->task_count = 0;
    return manager;
}

void scenario_manager_destroy(ScenarioManager* manager) {
    if (manager) {
        scenario_manager_clear_task_queue(manager);
        free(manager);
    }
}


// =============================================================================
// --- 7. D* Lite 경로 탐색 함수 구현 ---
// =============================================================================
static double heuristic(const Node* a, const Node* b) {
    return fabs(a->x - b->x) + fabs(a->y - b->y);
}

static double euclidean_distance(const Node* a, const Node* b) {
    return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2));
}

/**
 * @brief D* Lite의 Key 계산 함수. 로봇의 현재 위치와 k_m 값을 사용합니다.
 */
static Key calculate_key(const Pathfinder* pf, const Node* n) {
    double m = fmin(n->g, n->rhs);
    return (Key) { m + heuristic(n, pf->robot_pos) + pf->k_m, m };
}

static void path_update_vertex(Pathfinder* pf, GridMap* map, const AgentManager* agent_manager, Node* u) {
    if (u != pf->goal_node) {
        double min_rhs = INF;
        int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 };
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int nx = u->x + dx[i], ny = u->y + dy[i];
            if (grid_is_valid_coord(nx, ny)) {
                Node* successor = &map->grid[ny][nx];
                if (!grid_is_node_blocked(map, agent_manager, successor)) {
                    min_rhs = fmin(min_rhs, successor->g + 1.0);
                }
            }
        }
        u->rhs = min_rhs;
    }

    if (pq_contains(u)) {
        pq_remove(&pf->pq, u);
    }

    if (fabs(u->g - u->rhs) > 1e-9) {
        u->key = calculate_key(pf, u);
        pq_push(&pf->pq, u);
    }
}

Pathfinder* pathfinder_create(Node* robot_pos, Node* goal) {
    Pathfinder* pf = (Pathfinder*)malloc(sizeof(Pathfinder));
    if (!pf) return NULL;
    pf->robot_pos = robot_pos;
    pf->goal_node = goal;
    pf->k_m = 0.0; // k_m 초기화
    pq_init(&pf->pq, GRID_WIDTH * GRID_HEIGHT);
    return pf;
}

void pathfinder_destroy(Pathfinder* pf) {
    if (pf) {
        pq_free(&pf->pq);
        free(pf);
    }
}

void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, AgentManager* agent_manager) {
    // D* Lite에서는 g, rhs 값을 전체 초기화하지 않고 변경된 부분만 업데이트합니다.
    // 하지만 이 시뮬레이션에서는 매번 새로 계산하는 것이 간단하므로 전체 초기화 방식을 유지합니다。
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            Node* n = &map->grid[y][x];
            n->g = INF; n->rhs = INF; n->in_pq = FALSE; n->pq_index = -1;
        }
    }

    if (pf->goal_node) {
        pf->goal_node->rhs = 0;
        pf->goal_node->key = calculate_key(pf, pf->goal_node);
        pq_push(&pf->pq, pf->goal_node);
    }

    int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 };

    // D* Lite: 조건문이 로봇의 현재 위치(robot_pos)를 기준으로 변경됩니다。
    while (pf->pq.size > 0 &&
        (compare_keys(pq_top_key(&pf->pq), calculate_key(pf, pf->robot_pos)) < 0 ||
            fabs(pf->robot_pos->rhs - pf->robot_pos->g) > 1e-9)) {

        Node* u = pq_pop(&pf->pq);
        if (u->g > u->rhs) {
            u->g = u->rhs;
            for (int i = 0; i < NUM_DIRECTIONS; i++) {
                int px = u->x + dx[i], py = u->y + dy[i];
                if (grid_is_valid_coord(px, py)) path_update_vertex(pf, map, agent_manager, &map->grid[py][px]);
            }
        }
        else {
            u->g = INF;
            path_update_vertex(pf, map, agent_manager, u);
            for (int i = 0; i < NUM_DIRECTIONS; i++) {
                int px = u->x + dx[i], py = u->y + dy[i];
                if (grid_is_valid_coord(px, py)) path_update_vertex(pf, map, agent_manager, &map->grid[py][px]);
            }
        }
    }
}

Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager) {
    Node* current_node = pf->robot_pos;
    if (!pf->goal_node || current_node->g >= INF || current_node == pf->goal_node) {
        return current_node;
    }

    double min_g = INF;
    Node* best_next_node = current_node;
    double min_dist_to_goal = euclidean_distance(current_node, pf->goal_node);

    int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 };

    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        int nx = current_node->x + dx[i], ny = current_node->y + dy[i];
        if (grid_is_valid_coord(nx, ny)) {
            Node* neighbor = &map->grid[ny][nx];
            if (grid_is_node_blocked(map, agent_manager, neighbor)) continue;

            if (neighbor->g < min_g) {
                min_g = neighbor->g;
                best_next_node = neighbor;
                min_dist_to_goal = euclidean_distance(neighbor, pf->goal_node);
            }
            else if (fabs(neighbor->g - min_g) < 1e-9) {
                double dist_to_goal = euclidean_distance(neighbor, pf->goal_node);
                if (dist_to_goal < min_dist_to_goal) {
                    best_next_node = neighbor;
                    min_dist_to_goal = dist_to_goal;
                }
            }
        }
    }
    return best_next_node;
}


// =============================================================================
// --- 8. 에이전트 관리 및 로직 구현 ---
// =============================================================================
static double calculate_path_cost_for_selection(Node* start, Node* goal, GridMap* map, AgentManager* agent_manager) {
    Pathfinder* pf = pathfinder_create(start, goal);
    if (!pf) return INF;
    pathfinder_compute_shortest_path(pf, map, agent_manager);
    double cost = pf->robot_pos->g;
    pathfinder_destroy(pf);
    return cost;
}

static Node* select_best_parking_spot(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
    double best_cost = INF;
    Node* best_goal = NULL;
    for (int j = 0; j < map->num_goals; j++) {
        Node* g = map->goals[j];
        if (g->is_parked || (g->reserved_by_agent != -1 && g->reserved_by_agent != agent->id)) continue;

        double cost = calculate_path_cost_for_selection(agent->pos, g, map, agent_manager);
        if (cost < best_cost) {
            best_cost = cost;
            best_goal = g;
        }
    }
    if (best_goal) {
        logger_log(logger, "[%sPLAN%s] Agent %c, selected parking spot (%d,%d) (Cost: %.1f)", C_CYN, C_NRM, agent->symbol, best_goal->x, best_goal->y, best_cost);
    }
    return best_goal;
}

static Node* select_best_parked_car(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
    double best_cost = INF;
    Node* best_spot = NULL;
    for (int j = 0; j < map->num_goals; j++) {
        Node* g = map->goals[j];
        if (!g->is_parked || (g->reserved_by_agent != -1 && g->reserved_by_agent != agent->id)) continue;

        g->is_parked = FALSE;
        double cost = calculate_path_cost_for_selection(agent->pos, g, map, agent_manager);
        g->is_parked = TRUE;

        if (cost < best_cost) {
            best_cost = cost;
            best_spot = g;
        }
    }
    if (best_spot) {
        logger_log(logger, "[%sPLAN%s] Agent %c, selected car to collect at (%d,%d) (Cost: %.1f)", C_CYN, C_NRM, agent->symbol, best_spot->x, best_spot->y, best_cost);
    }
    return best_spot;
}

static Node* select_best_charge_station(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
    double best_cost = INF;
    Node* best_station = NULL;
    for (int i = 0; i < map->num_charge_stations; i++) {
        Node* station = map->charge_stations[i];
        if (station->reserved_by_agent != -1 && station->reserved_by_agent != agent->id) continue;
        double cost = calculate_path_cost_for_selection(agent->pos, station, map, agent_manager);
        if (cost < best_cost) {
            best_cost = cost;
            best_station = station;
        }
    }
    if (best_station) {
        logger_log(logger, "[%sPLAN%s] Agent %c, selected charge station (%d,%d) (Cost: %.1f)", C_CYN, C_NRM, agent->symbol, best_station->x, best_station->y, best_cost);
    }
    return best_station;
}

static void agent_set_goal(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
    if (agent->state == RETURNING_HOME_EMPTY && agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
        if (agent->goal) {
            agent->goal->reserved_by_agent = -1;
            agent->goal = NULL;
        }
        if (agent->pf) {
            pathfinder_destroy(agent->pf);
            agent->pf = NULL;
        }
        logger_log(logger, "[%sCHARGE%s] Agent %c needs charging! Overriding goal to charge station.", C_B_YEL, C_NRM, agent->symbol);
        agent->state = GOING_TO_CHARGE;
    }

    if (agent->state == IDLE || agent->state == CHARGING || agent->goal != NULL) return;

    switch (agent->state) {
    case GOING_TO_PARK: agent->goal = select_best_parking_spot(agent, map, agent_manager, logger); break;
    case RETURNING_HOME_EMPTY:
    case RETURNING_WITH_CAR:
    case RETURNING_HOME_MAINTENANCE:
        agent->goal = agent->home_base; break;
    case GOING_TO_COLLECT: agent->goal = select_best_parked_car(agent, map, agent_manager, logger); break;
    case GOING_TO_CHARGE: agent->goal = select_best_charge_station(agent, map, agent_manager, logger); break;
    default: break;
    }

    if (agent->goal) {
        agent->goal->reserved_by_agent = agent->id;
    }
    else if (agent->state != RETURNING_HOME_EMPTY && agent->state != RETURNING_WITH_CAR && agent->state != RETURNING_HOME_MAINTENANCE) {
        agent->state = IDLE;
        logger_log(logger, "[%sINFO%s] Agent %c: No available goal. Switching to IDLE.", C_YEL, C_NRM, agent->symbol);
    }
}

void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        next_pos[i] = manager->agents[i].pos;
    }

    // 우선순위 순서(ID 0, 1, 2) 대로 계획
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (agent->state == IDLE || agent->state == CHARGING || agent->state == AVOIDING_COLLISION) continue;

        agent_set_goal(agent, map, manager, logger);
        if (!agent->goal) continue;

        // D* Lite: 경로 탐색기가 없으면 새로 생성하고 초기 경로 계산
        if (!agent->pf) {
            agent->pf = pathfinder_create(agent->pos, agent->goal);
            pathfinder_compute_shortest_path(agent->pf, map, manager);
        }
        
        // 다른 에이전트들을 임시 장애물로 설정
        Node* obstacles_to_clear[MAX_AGENTS];
        int obs_count = 0;
        for (int j = 0; j < MAX_AGENTS; j++) {
            if (i == j) continue;
            Node* obs_node = (j < i) ? next_pos[j] : manager->agents[j].pos;
            if (obs_node && !obs_node->is_temp) {
                obs_node->is_temp = TRUE;
                obstacles_to_clear[obs_count++] = obs_node;
            }
        }
        
        // D* Lite: 경로 재계산(Repair)
        pathfinder_compute_shortest_path(agent->pf, map, manager);
        next_pos[i] = pathfinder_get_next_step(agent->pf, map, manager);

        // 임시 장애물 설정 해제
        for (int k = 0; k < obs_count; k++) {
            if (obstacles_to_clear[k]) obstacles_to_clear[k]->is_temp = FALSE;
        }
    }

    // 충돌 해결 로직 (기존과 동일)
    int resolved;
    for (int iter = 0; iter < MAX_AGENTS; iter++) {
        resolved = TRUE;
        for (int i = 0; i < MAX_AGENTS; i++) {
            for (int j = i + 1; j < MAX_AGENTS; j++) {
                if (next_pos[i] == next_pos[j] && next_pos[i] != manager->agents[i].pos) {
                    logger_log(logger, "[%sAVOID%s] Collision detected! Agent %c waits.", C_B_RED, C_NRM, manager->agents[j].symbol);
                    next_pos[j] = manager->agents[j].pos;
                    resolved = FALSE;
                }
                else if (next_pos[i] == manager->agents[j].pos && next_pos[j] == manager->agents[i].pos) {
                    logger_log(logger, "[%sAVOID%s] Head-on collision detected! Agent %c waits.", C_B_RED, C_NRM, manager->agents[j].symbol);
                    next_pos[j] = manager->agents[j].pos;
                    resolved = FALSE;
                }
            }
        }
        if (resolved) break;
    }
}


void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (agent->state == AVOIDING_COLLISION) {
            agent->state = IDLE;
        }
        if (agent->state == IDLE || agent->state == CHARGING || !agent->goal || agent->pos != agent->goal) {
            continue;
        }

        Node* reached_goal = agent->goal;
        if (agent->state != GOING_TO_CHARGE) {
            reached_goal->reserved_by_agent = -1;
        }
        agent->goal = NULL;
        
        // 목표에 도달했으므로 경로 탐색기 파괴
        if (agent->pf) {
            pathfinder_destroy(agent->pf);
            agent->pf = NULL;
        }

        switch (agent->state) {
        case GOING_TO_PARK:
            reached_goal->is_parked = TRUE;
            manager->total_cars_parked++;
            logger_log(logger, "[%sPARK%s] Agent %c, parking complete at (%d,%d).", C_GRN, C_NRM, agent->symbol, reached_goal->x, reached_goal->y);
            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == PARK_PHASE) {
                scenario->tasks_completed_in_phase++;
            }
            agent->state = RETURNING_HOME_EMPTY;
            break;

        case RETURNING_HOME_EMPTY:
            logger_log(logger, "[%sINFO%s] Agent %c, returned to base after parking.", C_CYN, C_NRM, agent->symbol);
            agent->state = IDLE;
            break;

        case GOING_TO_COLLECT:
            logger_log(logger, "[%sEXIT%s] Agent %c, collected car at (%d,%d).", C_YEL, C_NRM, agent->symbol, reached_goal->x, reached_goal->y);
            reached_goal->is_parked = FALSE;
            manager->total_cars_parked--;
            agent->state = RETURNING_WITH_CAR;
            break;

        case RETURNING_WITH_CAR:
            logger_log(logger, "[%sEXIT%s] Agent %c, car exit complete.", C_GRN, C_NRM, agent->symbol);
            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == EXIT_PHASE) {
                scenario->tasks_completed_in_phase++;
            }
            agent->state = IDLE;
            break;

        case GOING_TO_CHARGE:
            logger_log(logger, "[%sCHARGE%s] Agent %c, starting to charge. (%d steps)", C_B_YEL, C_NRM, agent->symbol, CHARGE_TIME);
            agent->state = CHARGING;
            agent->charge_timer = CHARGE_TIME;
            break;

        case RETURNING_HOME_MAINTENANCE:
            logger_log(logger, "[%sINFO%s] Agent %c, returned to base after charging.", C_CYN, C_NRM, agent->symbol);
            agent->state = IDLE;
            break;
        default: break;
        }
    }
}

void agent_manager_update_charge_state(AgentManager* manager, Logger* logger) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (agent->state == CHARGING) {
            agent->charge_timer--;
            if (agent->charge_timer <= 0) {
                logger_log(logger, "[%sCHARGE%s] Agent %c charging complete.", C_B_GRN, C_NRM, agent->symbol);
                agent->total_distance_traveled = 0.0;
                agent->state = RETURNING_HOME_MAINTENANCE;
                if (agent->pos) agent->pos->reserved_by_agent = -1;
                agent->goal = NULL;
            }
        }
    }
}

// =============================================================================
// --- 9. 시뮬레이션 관리 및 실행 ---
// =============================================================================
Simulation* simulation_create() {
    Simulation* sim = (Simulation*)calloc(1, sizeof(Simulation));
    if (!sim) { perror("Simulation allocation failed"); exit(1); }

    sim->agent_manager = agent_manager_create();
    sim->map = grid_map_create(sim->agent_manager);
    sim->scenario_manager = scenario_manager_create();
    sim->logger = logger_create();

    return sim;
}

void simulation_destroy(Simulation* sim) {
    if (sim) {
        grid_map_destroy(sim->map);
        agent_manager_destroy(sim->agent_manager);
        scenario_manager_destroy(sim->scenario_manager);
        logger_destroy(sim->logger);
        free(sim);
    }
}

static char get_single_char() {
#ifdef _WIN32
    return _getch();
#else
    char buf = 0;
    struct termios old = { 0 };
    fflush(stdout);
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");
    return buf;
#endif
}

static char get_char_input(const char* prompt, const char* valid_chars) {
    char choice;
    while (TRUE) {
        printf("%s", prompt);
        choice = tolower(get_single_char());
        printf("%c\n", choice);
        if (strchr(valid_chars, choice)) {
            return choice;
        }
        printf(C_B_RED "\nInvalid input. Please use one of the valid keys. (%s)\n" C_NRM, valid_chars);
    }
}

static int get_integer_input(const char* prompt, int min, int max) {
    char buffer[INPUT_BUFFER_SIZE];
    int value;
    while (TRUE) {
        printf("%s", prompt);
        if (fgets(buffer, sizeof(buffer), stdin) != NULL) {
            if (sscanf(buffer, "%d", &value) == 1 && value >= min && value <= max) return value;
        }
        printf(C_B_RED "Invalid input. Please enter an integer between %d and %d.\n" C_NRM, min, max);
    }
}

static float get_float_input(const char* prompt, float min, float max) {
    char buffer[INPUT_BUFFER_SIZE];
    float value;
    while (TRUE) {
        printf("%s", prompt);
        if (fgets(buffer, sizeof(buffer), stdin) != NULL) {
            if (sscanf(buffer, "%f", &value) == 1 && value >= min && value <= max) return value;
        }
        printf(C_B_RED "Invalid input. Please enter a number between %.1f and %.1f.\n" C_NRM, min, max);
    }
}

static int simulation_setup_custom_scenario(ScenarioManager* scenario) {
    printf(C_B_WHT "--- Custom Scenario Setup ---\n" C_NRM);
    scenario->num_phases = get_integer_input(C_YEL "Enter total number of phases (1-20, 0=cancel): " C_NRM, 0, MAX_PHASES);
    if (scenario->num_phases == 0) return 0;

    for (int i = 0; i < scenario->num_phases; i++) {
        printf(C_B_CYN "\n--- Phase %d/%d Setup ---\n" C_NRM, i + 1, scenario->num_phases);
        printf("a. %sPark%s\n", C_YEL, C_NRM);
        printf("b. %sExit%s\n", C_CYN, C_NRM);
        char type_choice = get_char_input("Select phase type: ", "ab");
        scenario->phases[i].task_count = get_integer_input("Enter number of cars for this phase: ", 1, 100);

        if (type_choice == 'a') {
            scenario->phases[i].type = PARK_PHASE;
            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "Park");
        }
        else {
            scenario->phases[i].type = EXIT_PHASE;
            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "Exit");
        }
        printf(C_GRN "Phase %d setup complete: %s %d cars.\n" C_NRM, i + 1, scenario->phases[i].type_name, scenario->phases[i].task_count);
    }
    printf(C_B_GRN "\n--- Scenario setup complete! ---\n" C_NRM);
    SLEEP(1500);
    return 1;
}

static int simulation_setup_realtime(ScenarioManager* scenario) {
    printf(C_B_WHT "--- Real-time Simulation Setup ---\n" C_NRM);
    while (TRUE) {
        scenario->park_chance = get_integer_input("\nEnter park request probability (0-100): ", 0, 100);
        scenario->exit_chance = get_integer_input("Enter exit request probability (0-100): ", 0, 100);
        if ((scenario->park_chance + scenario->exit_chance) <= 100) break;
        printf(C_B_RED "The sum of park and exit probabilities cannot exceed 100.\n" C_NRM);
    }
    printf(C_B_GRN "\nSetup complete: Park chance %d%%, Exit chance %d%%\n" C_NRM, scenario->park_chance, scenario->exit_chance);
    SLEEP(1500);
    return 1;
}

static int simulation_setup_speed(ScenarioManager* scenario) {
    printf(C_B_WHT "\n--- Simulation Speed Setup ---\n" C_NRM);
    scenario->speed_multiplier = get_float_input("Enter desired speed multiplier (1.0 to 100.0): ", 1.0f, MAX_SPEED_MULTIPLIER);
    scenario->simulation_speed = (int)(100.0f / scenario->speed_multiplier);
    if (scenario->simulation_speed < 1) scenario->simulation_speed = 1;
    printf(C_B_GRN "\n--- Starting simulation at %.1fx speed... ---\n" C_NRM, scenario->speed_multiplier);
    SLEEP(1500);
    return 1;
}

int simulation_setup(Simulation* sim) {
    ui_clear_screen_optimized();
    printf(C_B_WHT "--- Simulation Mode Selection ---\n" C_NRM);
    printf("a. %sCustom Scenario%s\n", C_YEL, C_NRM);
    printf("b. %sReal-time Simulation%s\n", C_CYN, C_NRM);
    printf("q. %sQuit%s\n\n", C_RED, C_NRM);

    char choice = get_char_input("Enter scenario character to run: ", "abq");
    int setup_success = 0;
    switch (choice) {
    case 'a':
        sim->scenario_manager->mode = MODE_CUSTOM;
        if (simulation_setup_custom_scenario(sim->scenario_manager)) {
            setup_success = simulation_setup_speed(sim->scenario_manager);
        }
        break;
    case 'b':
        sim->scenario_manager->mode = MODE_REALTIME;
        if (simulation_setup_realtime(sim->scenario_manager)) {
            setup_success = simulation_setup_speed(sim->scenario_manager);
        }
        break;
    case 'q':
        return 0;
    }

    if (setup_success) {
        ui_clear_screen_optimized();
    }
    return setup_success;
}

static void add_task_to_queue(ScenarioManager* scenario, TaskType type) {
    if (scenario->task_count >= MAX_TASKS) return;

    TaskNode* new_task = (TaskNode*)malloc(sizeof(TaskNode));
    if (!new_task) {
        perror("Failed to allocate memory for new task");
        return;
    }
    new_task->type = type;
    new_task->next = NULL;

    if (scenario->task_queue_head == NULL) {
        scenario->task_queue_head = new_task;
        scenario->task_queue_tail = new_task;
    }
    else {
        scenario->task_queue_tail->next = new_task;
        scenario->task_queue_tail = new_task;
    }
    scenario->task_count++;
}

void agent_manager_assign_tasks_to_agents(AgentManager* manager, ScenarioManager* scenario, GridMap* map, Logger* logger) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (agent->state == IDLE && agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
            if (select_best_charge_station(agent, map, manager, logger)) {
                agent->state = GOING_TO_CHARGE;
            }
            else {
                logger_log(logger, "[%sWARN%s] Agent %c needs charging but all stations are busy.", C_YEL, C_NRM, agent->symbol);
            }
        }
    }

    if (scenario->mode == MODE_REALTIME) {
        TaskNode* current_task = scenario->task_queue_head;
        TaskNode* prev = NULL;
        while (current_task != NULL) {
            Agent* idle_agent = NULL;
            for (int i = 0; i < MAX_AGENTS; ++i) {
                if (manager->agents[i].state == IDLE) {
                    idle_agent = &manager->agents[i];
                    break;
                }
            }

            if (!idle_agent) break;

            int task_assigned = FALSE;
            if (current_task->type == TASK_PARK && manager->total_cars_parked < map->num_goals) {
                idle_agent->state = GOING_TO_PARK;
                logger_log(logger, "[%sTASK%s] Agent %c assigned a new PARK task.", C_CYN, C_NRM, idle_agent->symbol);
                task_assigned = TRUE;
            }
            else if (current_task->type == TASK_EXIT && manager->total_cars_parked > 0) {
                idle_agent->state = GOING_TO_COLLECT;
                logger_log(logger, "[%sTASK%s] Agent %c assigned a new EXIT task.", C_CYN, C_NRM, idle_agent->symbol);
                task_assigned = TRUE;
            }

            if (task_assigned) {
                TaskNode* to_free = current_task;
                if (prev == NULL) {
                    scenario->task_queue_head = current_task->next;
                }
                else {
                    prev->next = current_task->next;
                }
                if (current_task == scenario->task_queue_tail) {
                    scenario->task_queue_tail = prev;
                }
                current_task = current_task->next;
                free(to_free);
                scenario->task_count--;
            }
            else {
                prev = current_task;
                current_task = current_task->next;
            }
        }
    }
}


static void simulation_update_state(Simulation* sim) {
    ScenarioManager* scenario = sim->scenario_manager;
    AgentManager* agent_manager = sim->agent_manager;
    GridMap* map = sim->map;
    Logger* logger = sim->logger;

    if (scenario->mode == MODE_CUSTOM) {
        if (scenario->current_phase_index >= scenario->num_phases) return;
        DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
        if (scenario->tasks_completed_in_phase >= phase->task_count) {
            logger_log(logger, "[%sPHASE%s] Phase %d (%s %d) complete!", C_B_YEL, C_NRM, scenario->current_phase_index + 1, phase->type_name, phase->task_count);
            scenario->current_phase_index++;
            scenario->tasks_completed_in_phase = 0;
            if (scenario->current_phase_index < scenario->num_phases) {
                DynamicPhase* next_phase = &scenario->phases[scenario->current_phase_index];
                logger_log(logger, "[%sPHASE%s] Starting Phase %d: %s %d.", C_B_YEL, C_NRM, scenario->current_phase_index + 1, next_phase->type_name, next_phase->task_count);
                SLEEP(1500);
            }
            return;
        }
    }
    else if (scenario->mode == MODE_REALTIME) {
        if (scenario->time_step > 0 && scenario->time_step % EVENT_GENERATION_INTERVAL == 0) {
            int event_chance = rand() % 100;
            if (event_chance < scenario->park_chance && agent_manager->total_cars_parked < map->num_goals) {
                logger_log(logger, "[%sEVENT%s] New park request generated.", C_B_GRN, C_NRM);
                add_task_to_queue(scenario, TASK_PARK);
            }
            else if (event_chance < (scenario->park_chance + scenario->exit_chance) && agent_manager->total_cars_parked > 0) {
                logger_log(logger, "[%sEVENT%s] New exit request generated.", C_B_YEL, C_NRM);
                add_task_to_queue(scenario, TASK_EXIT);
            }
        }
    }

    agent_manager_assign_tasks_to_agents(agent_manager, scenario, map, logger);
}

static int grid_map_render_to_buffer(char* buffer, size_t buffer_size, const GridMap* map, const AgentManager* agent_manager) {
    char view[GRID_HEIGHT][GRID_WIDTH];
    const char* colors[GRID_HEIGHT][GRID_WIDTH];
    char* buf_ptr = buffer;
    size_t remaining_size = buffer_size;
    int written = 0;

    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (map->grid[y][x].is_obstacle) {
                view[y][x] = '+'; colors[y][x] = C_WHT;
            }
            else {
                view[y][x] = '.'; colors[y][x] = C_GRY;
            }
        }
    }

    for (int i = 0; i < map->num_charge_stations; i++) {
        Node* cs = map->charge_stations[i];
        view[cs->y][cs->x] = 'E';
        int is_charging_here = FALSE;
        for (int j = 0; j < MAX_AGENTS; j++) {
            if (agent_manager->agents[j].state == CHARGING && agent_manager->agents[j].pos == cs) {
                is_charging_here = TRUE; break;
            }
        }
        colors[cs->y][cs->x] = is_charging_here ? C_B_RED : C_B_YEL;
    }
    for (int i = 0; i < map->num_goals; i++) {
        Node* g = map->goals[i];
        if (g->is_parked) {
            view[g->y][g->x] = 'P'; colors[g->y][g->x] = C_RED;
        }
        else if (g->is_goal) {
            view[g->y][g->x] = 'G'; colors[g->y][g->x] = C_GRN;
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        if (agent_manager->agents[i].pos) {
            view[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = agent_manager->agents[i].symbol;
            if (i == 0) colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_CYN;
            else if (i == 1) colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_YEL;
            else colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_MAG;
        }
    }

    written = snprintf(buf_ptr, remaining_size, C_B_WHT "\n--- D* Lite Parking Simulation ---\n" C_NRM);
    buf_ptr += written; remaining_size -= written;
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            written = snprintf(buf_ptr, remaining_size, "%s %c" C_NRM, colors[y][x], view[y][x]);
            buf_ptr += written; remaining_size -= written;
        }
        written = snprintf(buf_ptr, remaining_size, "\n");
        buf_ptr += written; remaining_size -= written;
    }
    written = snprintf(buf_ptr, remaining_size, "\n");
    buf_ptr += written; remaining_size -= written;

    return (int)(buf_ptr - buffer);
}


static void simulation_display_status(const Simulation* sim) {
    const ScenarioManager* scenario = sim->scenario_manager;
    const AgentManager* agent_manager = sim->agent_manager;
    const GridMap* map = sim->map;
    const Logger* logger = sim->logger;
    char display_buffer[DISPLAY_BUFFER_SIZE];
    char* buf_ptr = display_buffer;
    size_t remaining_size = sizeof(display_buffer);
    int written = 0;

    written = snprintf(buf_ptr, remaining_size, C_B_WHT);
    buf_ptr += written; remaining_size -= written;

    if (scenario->mode == MODE_CUSTOM) {
        if (scenario->current_phase_index < scenario->num_phases) {
            DynamicPhase* p = &scenario->phases[scenario->current_phase_index];
            written = snprintf(buf_ptr, remaining_size, "--- Custom Scenario: %d/%d [Speed: %.1fx] ---\n", scenario->current_phase_index + 1, scenario->num_phases, scenario->speed_multiplier);
            buf_ptr += written; remaining_size -= written;
            written = snprintf(buf_ptr, remaining_size, "Time: %d, Current Task: %s (%d/%d)\n", scenario->time_step, p->type_name, scenario->tasks_completed_in_phase, p->task_count);
            buf_ptr += written; remaining_size -= written;
        }
        else {
            written = snprintf(buf_ptr, remaining_size, "--- Custom Scenario: All phases complete ---\n");
            buf_ptr += written; remaining_size -= written;
        }
    }
    else if (scenario->mode == MODE_REALTIME) {
        written = snprintf(buf_ptr, remaining_size, "--- Real-time Simulation [Speed: %.1fx] ---\n", scenario->speed_multiplier);
        buf_ptr += written; remaining_size -= written;

        int park_requests = 0;
        int exit_requests = 0;
        TaskNode* current_task = scenario->task_queue_head;
        while (current_task != NULL) {
            if (current_task->type == TASK_PARK) park_requests++;
            else if (current_task->type == TASK_EXIT) exit_requests++;
            current_task = current_task->next;
        }

        written = snprintf(buf_ptr, remaining_size, "Time: %d / %d | Pending Tasks: %d (%sPark Req: %d%s, %sExit Req: %d%s)\n",
            scenario->time_step, REALTIME_MODE_TIMELIMIT, scenario->task_count,
            C_B_GRN, park_requests, C_NRM, C_B_YEL, exit_requests, C_NRM);
        buf_ptr += written; remaining_size -= written;
    }
    written = snprintf(buf_ptr, remaining_size, "Parked Cars: %d/%d\n" C_NRM, agent_manager->total_cars_parked, map->num_goals);
    buf_ptr += written; remaining_size -= written;

    written = grid_map_render_to_buffer(buf_ptr, remaining_size, map, agent_manager);
    buf_ptr += written; remaining_size -= written;

    const char* agent_state_strings[] = { "IDLE", "GOING_TO_PARK", "RETURNING_HOME_EMPTY", "GOING_TO_COLLECT", "RETURNING_WITH_CAR", "GOING_TO_CHARGE", "CHARGING", "RETURNING_HOME_MAINTENANCE", "AVOIDING_COLLISION" };
    const char* agent_state_colors[] = { C_GRY, C_YEL, C_CYN, C_YEL, C_GRN, C_B_RED, C_RED, C_CYN, C_B_RED };

    for (int i = 0; i < MAX_AGENTS; i++) {
        const Agent* agent = &agent_manager->agents[i];
        const char* agent_color = (i == 0) ? C_B_CYN : (i == 1) ? C_B_YEL : C_B_MAG;
        char status_buffer[100];

        if (agent->state == CHARGING) {
            snprintf(status_buffer, sizeof(status_buffer), "CHARGING... (%d)", agent->charge_timer);
        }
        else {
            snprintf(status_buffer, sizeof(status_buffer), "%s", agent_state_strings[agent->state]);
        }

        written = snprintf(buf_ptr, remaining_size, "%sAgent %c%s: (%2d,%d) ", agent_color, agent->symbol, C_NRM, agent->pos->x, agent->pos->y);
        buf_ptr += written; remaining_size -= written;

        if (agent->goal) {
            written = snprintf(buf_ptr, remaining_size, "-> (%2d,%d) ", agent->goal->x, agent->goal->y);
        }
        else {
            written = snprintf(buf_ptr, remaining_size, "-> None      ");
        }
        buf_ptr += written; remaining_size -= written;

        written = snprintf(buf_ptr, remaining_size, "[Mileage: %6.1f/%d] [%s%-*s%s]\n",
            agent->total_distance_traveled, (int)DISTANCE_BEFORE_CHARGE,
            agent_state_colors[agent->state], STATUS_STRING_WIDTH, status_buffer, C_NRM);
        buf_ptr += written; remaining_size -= written;
    }
    written = snprintf(buf_ptr, remaining_size, "\n");
    buf_ptr += written; remaining_size -= written;

    written = snprintf(buf_ptr, remaining_size, C_B_WHT "--- Simulation Log ---\n" C_NRM);
    buf_ptr += written; remaining_size -= written;
    for (int i = 0; i < logger->log_count; i++) {
        int index = (logger->log_head + i) % LOG_BUFFER_LINES;
        written = snprintf(buf_ptr, remaining_size, "%s%s%s\n", C_GRY, logger->logs[index], C_NRM);
        buf_ptr += written; remaining_size -= written;
    }

    ui_clear_screen_optimized();
    printf("%s", display_buffer);
}


static int simulation_is_complete(const Simulation* sim) {
    const ScenarioManager* scenario = sim->scenario_manager;
    const AgentManager* agent_manager = sim->agent_manager;

    if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index >= scenario->num_phases) {
        for (int i = 0; i < MAX_AGENTS; i++) {
            if (agent_manager->agents[i].state != IDLE) return FALSE;
        }
        printf(C_B_GRN "\nAll scenario phases are complete! Shutting down simulation.\n" C_NRM);
        return TRUE;
    }

    if (scenario->mode == MODE_REALTIME && scenario->time_step >= REALTIME_MODE_TIMELIMIT) {
        printf(C_B_GRN "\nReal-time simulation time limit reached! Shutting down simulation.\n" C_NRM);
        return TRUE;
    }
    return FALSE;
}

void simulation_run(Simulation* sim) {
    while (TRUE) {
        // 1. 에이전트 상태 업데이트 (충전, 작업 할당)
        agent_manager_update_charge_state(sim->agent_manager, sim->logger);
        simulation_update_state(sim);

        // 2. D* Lite 경로 계획 및 충돌 해결
        Node* next_pos[MAX_AGENTS];
        agent_manager_plan_and_resolve_collisions(sim->agent_manager, sim->map, sim->logger, next_pos);

        // 3. 에이전트 이동 및 D* Lite 상태 업데이트
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* agent = &sim->agent_manager->agents[i];
            if (agent->state != CHARGING && next_pos[i] != NULL) {
                if (agent->pos != next_pos[i]) {
                    Node* last_pos = agent->pos;
                    agent->total_distance_traveled += 1.0;
                    agent->pos = next_pos[i];

                    // D* Lite: 이동 후 상태 업데이트
                    if (agent->pf) {
                        agent->pf->k_m += heuristic(last_pos, agent->pos);
                        agent->pf->robot_pos = agent->pos;
                    }
                }
            }
        }
        
        // 4. 이동 후 목표 도달 여부 확인
        agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->logger);

        // 5. 화면 표시
        simulation_display_status(sim);

        // 6. 시뮬레이션 종료 조건 확인
        if (simulation_is_complete(sim)) break;

        sim->scenario_manager->time_step++;
        SLEEP(sim->scenario_manager->simulation_speed);
    }
}

// =============================================================================
// --- 10. 메인 함수 ---
// =============================================================================
int main() {
    srand((unsigned int)time(NULL));
    system_enable_virtual_terminal();

    Simulation* sim = simulation_create();
    if (!sim) return 1;

    if (simulation_setup(sim)) {
        simulation_run(sim);
    }
    else {
        printf("\nSimulation cancelled. Exiting program.\n");
    }

    simulation_destroy(sim);
    return 0;
}
