///**
//* @file lpa_parking.c
//* @brief lpa* �˰��� ��� 3-������Ʈ �ڵ� ���� �ùķ��̼� (���� ����Ʈ ť �� ������ �۾� ó�� ���� ����)
//*/
//#define _CRT_SECURE_NO_WARNINGS // visual studio ȯ�濡�� ���� ��� ��Ȱ��ȭ�ϱ� ���� ��ũ��
//#include <stdio.h>      // ǥ�� ����� �Լ�(printf, scanf ��)�� ����ϱ� ���� ��� ����
//#include <stdlib.h>     // ���� �޸� �Ҵ�(malloc, free), ���� ����(rand) ���� ���� ��� ����
//#include <string.h>     // ���ڿ� ó�� �Լ�(strcpy, strcmp ��)�� ����ϱ� ���� ��� ����
//#include <math.h>       // ���� �Լ�(sqrt, fabs ��)�� ����ϱ� ���� ��� ����
//#include <float.h>      // �ε��Ҽ��� ���� �ִ밪(DBL_MAX) �� �Ѱ谪�� ������ ��� ����
//#include <time.h>       // �ð� ���� �Լ�(time)�� ����ϱ� ���� ��� ���� (���� �õ� ������ ���)
//#include <stdarg.h>     // ���� ���� �Լ�(vsnprintf ��)�� ó���ϱ� ���� ��� ����
//#include <ctype.h>      // ���� ó�� �Լ�(tolower)�� ����ϱ� ���� ��� ����
//
//#ifdef _WIN32 // �ü���� windows�� ���
//#include <windows.h>    // windows api �Լ�(Sleep, SetConsoleCursorPosition ��)�� ����ϱ� ���� ��� ����
//#include <conio.h>      // �ܼ� ����� �Լ�(_getch)�� ����ϱ� ���� ��� ����
//#define sleep(ms) Sleep(ms) // Sleep �Լ��� �и��� ������ ����
//#else // windows�� �ƴ� �ٸ� �ü��(linux, macos ��)�� ���
//#include <unistd.h>     // posix �ü�� api(usleep, read)�� ����ϱ� ���� ��� ����
//#include <termios.h>    // �͹̳� ����(���� ��Ȱ��ȭ ��)�� ���� ��� ����
//#define sleep(ms) usleep(ms * 1000) // usleep �Լ��� �и��� ������ ���� (usleep�� ����ũ���� ����)
//#endif
//
//// =============================================================================
//// --- 1. ��� �� ���� ���� ���� ---
//// =============================================================================
//// --- �ý��� ��� ---
//#define TRUE 1                  // �Ҹ��� ��(true) �� ����
//#define FALSE 0                 // �Ҹ��� ����(false) �� ����
//#define INPUT_BUFFER_SIZE 100   // ����� �Է� ������ ũ�� ����
//#define DISPLAY_BUFFER_SIZE 16384 // ȭ�� �������� ���� ���� ũ�� ���� (���� ����ȭ��)
//
//// --- ANSI �÷� �ڵ� ---
//// �͹̳� �ؽ�Ʈ ������ �����ϱ� ���� ANSI �̽������� �ڵ� ����
//#define C_NRM "\x1b[0m"       // �⺻ �������� ����
//#define C_RED "\x1b[31m"       // ������
//#define C_GRN "\x1b[32m"       // �ʷϻ�
//#define C_YEL "\x1b[33m"       // �����
//#define C_BLU "\x1b[34m"       // �Ķ���
//#define C_MAG "\x1b[35m"       // ����Ÿ��
//#define C_CYN "\x1b[36m"       // �þȻ�
//#define C_WHT "\x1b[37m"       // ���
//#define C_GRY "\x1b[90m"       // ȸ��
//#define C_B_RED "\x1b[1;31m"   // ���� ������ (����)
//#define C_B_GRN "\x1b[1;32m"   // ���� �ʷϻ� (����)
//#define C_B_YEL "\x1b[1;33m"   // ���� ����� (����)
//#define C_B_MAG "\x1b[1;35m"   // ���� ����Ÿ�� (����)
//#define C_B_CYN "\x1b[1;36m"   // ���� �þȻ� (����)
//#define C_B_WHT "\x1b[1;37m"   // ���� ��� (����)
//
//// --- �ùķ��̼� �׸��� �� ������Ʈ ��� ---
//#define GRID_WIDTH 37           // �׸��� ���� �ʺ�
//#define GRID_HEIGHT 12          // �׸��� ���� ����
//#define MAX_AGENTS 3            // �ùķ��̼ǿ� �����ϴ� �ִ� ������Ʈ(�κ�) ��
//#define MAX_GOALS (GRID_WIDTH * GRID_HEIGHT) // ������ �ִ� ��ǥ ����(���� ����) ��
//#define INF DBL_MAX             // ���Ѵ� ���� double�� �ִ밪���� ���� (��� ��꿡 ���)
//#define NUM_DIRECTIONS 4        // ������Ʈ�� �̵��� �� �ִ� ������ �� (��, ��, ��, ��)
//
//// --- �ùķ��̼� ���� ��� ---
//#define DISTANCE_BEFORE_CHARGE 300.0 // ������ �ʿ������� ���� �̵� �Ÿ�
//#define CHARGE_TIME 20               // ������ �ɸ��� �ð� (�ùķ��̼� ���� ����)
//#define MAX_CHARGE_STATIONS 10       // �ִ� ������ ����
//#define MAX_PHASES 20                // ����� ���� �ó��������� ���� ������ �ִ� �ܰ� ��
//#define REALTIME_MODE_TIMELIMIT 10000 // �ǽð� ����� �ִ� �ùķ��̼� �ð�
//#define MAX_TASKS 50                 // �۾� ť�� ������ �� �ִ� �ִ� �۾� ��
//#define MAX_SPEED_MULTIPLIER 100.0f  // �ùķ��̼� �ִ� ���
//#define EVENT_GENERATION_INTERVAL 5 // �ǽð� ��忡�� �̺�Ʈ(����/���� ��û)�� �߻��ϴ� �ð� ����
//
//// --- �α� �� UI ��� ---
//#define LOG_BUFFER_LINES 5      // ȭ�鿡 ǥ���� �α� �޽����� �ִ� �� ��
//#define LOG_BUFFER_WIDTH 256    // �� �α� �޽����� �ִ� ����
//#define STATUS_STRING_WIDTH 25  // ������Ʈ ���� ���ڿ��� ����� ���� ���� �ʺ� (UI ���Ŀ�)
//
//// =============================================================================
//// --- 2. ����ü ���� ---
//// =============================================================================
//// --- �⺻ ����ü ---
///** @struct Key
// * @brief �켱���� ť(priority queue)���� ����� �켱������ �����ϴ� Ű ��.
// * LPA* �˰��򿡼� ���Ǹ�, k1�� k2 �� ������ �켱������ ���մϴ�.
// */
//typedef struct { double k1; double k2; } Key;
//
///** @struct Node
// * @brief �׸��� ���� �� ��(���)�� ��Ÿ���� ����ü.
// */
//typedef struct Node {
//    int x, y;                   // ����� �׸��� �� x, y ��ǥ
//    double g, rhs;              // LPA* �˰��򿡼� ����ϴ� g��(���� ���)�� rhs��(���� ���)
//    Key key;                    // �켱���� ť������ ������ ���� Ű ��
//    int is_obstacle;            // �� ��尡 ��ֹ����� ���� (TRUE/FALSE)
//    int is_goal;                // �� ��尡 ���� ����(��ǥ)���� ����
//    int is_temp;                // �ӽ� ��ֹ��� �����Ǿ����� ���� (�ٸ� ������Ʈ ȸ�ǿ�)
//    int is_parked;              // �� ��忡 ������ �����Ǿ� �ִ��� ����
//    int reserved_by_agent;      // � ������Ʈ�� ���� ����Ǿ����� (������Ʈ ID, -1�� ���� ����)
//    int in_pq;                  // ���� �켱���� ť�� ���ԵǾ� �ִ��� ����
//    int pq_index;               // �켱���� ť(��) �������� �ε���
//} Node;
//
///** @enum AgentState
// * @brief ������Ʈ�� ���� ���¸� ��Ÿ���� ������.
// */
//typedef enum {
//    IDLE,                       // ��� ����
//    GOING_TO_PARK,              // �����Ϸ� ���� ��
//    RETURNING_HOME_EMPTY,       // ���� �� �� ������ ������ �����ϴ� ��
//    GOING_TO_COLLECT,           // ������ ���� ������ ���� ��
//    RETURNING_WITH_CAR,         // ���� �ư� ���� �������� ���� ��
//    GOING_TO_CHARGE,            // �����ҷ� ���� ��
//    CHARGING,                   // ���� ��
//    RETURNING_HOME_MAINTENANCE  // ���� �� ������ �����ϴ� ����
//} AgentState;
//
///** @struct Agent
// * @brief �ϳ��� ������Ʈ(�κ�)�� ��Ÿ���� ����ü.
// */
//typedef struct {
//    int id;                     // ������Ʈ�� ���� ID (0, 1, 2)
//    char symbol;                // �ʿ� ǥ�õ� ������Ʈ�� ��ȣ ('A', 'B', 'C')
//    Node* pos;                  // ���� ��ġ�� ����Ű�� ��� ������
//    Node* home_base;            // ������Ʈ�� ����(������)�� ����Ű�� ���
//    Node* goal;                 // ���� ��ǥ ������ ����Ű�� ��� ������
//    AgentState state;           // ������Ʈ�� ���� ����
//    double total_distance_traveled; // �� �̵� �Ÿ� (���� �ʿ� ���� �Ǵܿ� ���)
//    int charge_timer;           // ���� �ܿ� �ð�
//} Agent;
//
///** @struct PriorityQueue
// * @brief �ּ� ��(min-heap)���� ������ �켱���� ť.
// * LPA* �˰��򿡼� ����� ���� ���� ��带 ȿ�������� ã�� ���� ���˴ϴ�.
// */
//typedef struct {
//    Node** nodes;   // ��� �����͸� �����ϴ� �迭
//    int size;       // ���� ť�� ����� ����� ����
//    int capacity;   // ť�� �ִ� �뷮
//} PriorityQueue;
//
///** @enum PhaseType
// * @brief ����� ���� �ó������� �� �ܰ� ������ ��Ÿ���� ������.
// */
//typedef enum {
//    PARK_PHASE, // ���� �ܰ�
//    EXIT_PHASE  // ���� �ܰ�
//} PhaseType;
//
///** @struct DynamicPhase
// * @brief ����� ���� �ó������� �� �ܰ踦 �����ϴ� ����ü.
// */
//typedef struct {
//    PhaseType type;             // �ܰ��� ���� (���� �Ǵ� ����)
//    int task_count;             // �� �ܰ迡�� �����ؾ� �� �۾�(����)�� ��
//    char type_name[10];         // �ܰ� ������ ���ڿ��� ���� (UI ǥ�ÿ�)
//} DynamicPhase;
//
///** @enum TaskType
// * @brief �ǽð� ��忡�� �����Ǵ� �۾��� ������ ��Ÿ���� ������.
// */
//typedef enum {
//    TASK_NONE,  // �۾� ����
//    TASK_PARK,  // ���� �۾�
//    TASK_EXIT   // ���� �۾�
//} TaskType;
//
///** @struct TaskNode
// * @brief �۾� ť�� ���� ��带 ��Ÿ���� ���� ����Ʈ ����ü.
// */
//typedef struct TaskNode {
//    TaskType type;
//    struct TaskNode* next;
//} TaskNode;
//
///** @enum SimulationMode
// * @brief �ùķ��̼��� ��ü ���� ��带 ��Ÿ���� ������.
// */
//typedef enum {
//    MODE_UNINITIALIZED, // �ʱ�ȭ���� ���� ����
//    MODE_CUSTOM,        // ����� ���� �ó����� ���
//    MODE_REALTIME       // �ǽð� ���
//} SimulationMode;
//
//// --- ���ȭ�� ����ü ---
///** @struct GridMap
//* @brief �׸���, ���� ����, ������ �� �� ���� �����͸� ����.
//*/
//typedef struct {
//    Node grid[GRID_HEIGHT][GRID_WIDTH]; // 2D �׸��� ��
//    Node* goals[MAX_GOALS];             // ��� ���� ���� ��带 ����Ű�� ������ �迭
//    int num_goals;                      // �� ���� ������ ��
//    Node* charge_stations[MAX_CHARGE_STATIONS]; // ��� ������ ��带 ����Ű�� ������ �迭
//    int num_charge_stations;            // �� �������� ��
//} GridMap;
//
///** @struct AgentManager
//* @brief ��� ������Ʈ�� ���¿� ������ ���� ���� ����.
//*/
//typedef struct {
//    Agent agents[MAX_AGENTS];   // ������Ʈ ��ü �迭
//    int total_cars_parked;      // ���� �����忡 ������ �� ���� ��
//} AgentManager;
//
///** @struct Pathfinder
//* @brief LPA* ��� Ž�� ���� ����(PQ, ����/��ǥ)�� �ӽ÷� ����.
//*/
//typedef struct {
//    PriorityQueue pq;           // ��� Ž���� ���� �켱���� ť
//    Node* start_node;           // Ž�� ���� ���
//    Node* goal_node;            // Ž�� ��ǥ ���
//} Pathfinder;
//
///** @struct ScenarioManager
//* @brief �ùķ��̼� ���, �ó����� �ܰ�, �۾� ť ���� ����.
//*/
//typedef struct {
//    SimulationMode mode;                // ���� �ùķ��̼� ���
//    int time_step;                      // �ùķ��̼� ��� �ð� (����)
//    int simulation_speed;               // �ùķ��̼� ���� �ð� (ms)
//    float speed_multiplier;             // �ùķ��̼� ���
//    DynamicPhase phases[MAX_PHASES];    // ����� ���� �ó������� �ܰ��
//    int num_phases;                     // �� �ܰ� ��
//    int current_phase_index;            // ���� ���� ���� �ܰ��� �ε���
//    int tasks_completed_in_phase;       // ���� �ܰ迡�� �Ϸ�� �۾� ��
//    TaskNode* task_queue_head;          // �۾� ť(���� ����Ʈ)�� ����
//    TaskNode* task_queue_tail;          // �۾� ť(���� ����Ʈ)�� ��
//    int task_count;                     // ť�� �ִ� �۾��� ��
//    int park_chance;                    // �ǽð� ��忡�� ���� ��û�� �߻��� Ȯ��
//    int exit_chance;                    // �ǽð� ��忡�� ���� ��û�� �߻��� Ȯ��
//} ScenarioManager;
//
///** @struct Logger
//* @brief ��ȯ �α� ���۸� ����.
//*/
//typedef struct {
//    char logs[LOG_BUFFER_LINES][LOG_BUFFER_WIDTH]; // �α� �޽����� �����ϴ� 2D �迭
//    int log_head;                       // ���� ������ �α��� �ε��� (��ȯ ���ۿ�)
//    int log_count;                      // ���� ����� �α��� ��
//} Logger;
//
///** @struct Simulation
//* @brief ��� ���� ����� �����ϰ� ��ü �ùķ��̼��� ���ɽ�Ʈ���̼�.
//*/
//typedef struct {
//    GridMap* map;                   // �� ������ ���� ���
//    AgentManager* agent_manager;    // ������Ʈ ���� ���
//    ScenarioManager* scenario_manager; // �ó����� ���� ���
//    Logger* logger;                 // �α� ���� ���
//} Simulation;
//
//// =============================================================================
//// --- 3. �Լ� ������Ÿ�� ���� ---
//// =============================================================================
//// --- ��� ����/�Ҹ� �Լ� ---
//Simulation* simulation_create(); // Simulation ��ü�� �����ϰ� �ʱ�ȭ
//void simulation_destroy(Simulation* sim); // Simulation ��ü�� ���� ������ �޸𸮸� ����
//
//// --- �ý���, UI �� �α� �Լ� ---
//void system_enable_virtual_terminal(); // Windows���� ANSI �÷� �ڵ带 ��� �����ϰ� ����
//void ui_clear_screen_optimized(); // ȭ���� ����� ����ȭ�� �Լ�
//int simulation_setup(Simulation* sim); // �ùķ��̼� ���� �� ���, �ó����� ���� ����
//void logger_log(Logger* logger, const char* format, ...); // �α� ���ۿ� �޽��� �߰�
//
//// --- �켱���� ť (Priority Queue) �Լ� ---
//void pq_init(PriorityQueue* pq, int capacity); // �켱���� ť �ʱ�ȭ
//void pq_free(PriorityQueue* pq); // �켱���� ť �޸� ����
//void pq_push(PriorityQueue* pq, Node* node); // ť�� ��� �߰�
//Node* pq_pop(PriorityQueue* pq); // ť���� �켱������ ���� ���� ��� ���� �� ��ȯ
//void pq_remove(PriorityQueue* pq, Node* node); // ť���� Ư�� ��� ����
//int pq_contains(const Node* node); // ��尡 ť�� ���ԵǾ� �ִ��� Ȯ��
//Key pq_top_key(const PriorityQueue* pq); // ť�� �ֻ��� ����� Ű ���� Ȯ�� (�������� ����)
//
//// --- �׸��� (GridMap) ���� �Լ� ---
//int grid_is_valid_coord(int x, int y); // �־��� ��ǥ�� �׸��� ���� ���� �ִ��� Ȯ��
//int grid_is_node_blocked(const GridMap* map, const AgentManager* agent_manager, const Node* n); // ��尡 �̵� �Ұ�������(��ֹ�, ������ �� ��) Ȯ��
//
//// --- ��� Ž�� (Pathfinder) �Լ� ---
//Pathfinder* pathfinder_create(Node* start, Node* goal); // ��� Ž���� ����
//void pathfinder_destroy(Pathfinder* pf); // ��� Ž���� �޸� ����
//void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, AgentManager* agent_manager); // LPA* �˰������� �ִ� ��� ���
//Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager, Node* current_node); // ���� ��θ� ���� ���� �̵��� ��� ��ȯ
//
//// --- ������Ʈ (AgentManager) ���� �Լ� ---
//void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]); // ��� ������Ʈ�� ���� �������� ��ȹ�ϰ� �浹 �ذ�
//void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger); // �̵� �� ������Ʈ ���� ������Ʈ (��ǥ ���� ��)
//void agent_manager_update_charge_state(AgentManager* manager, Logger* logger); // ���� ���� ������Ʈ ���� ������Ʈ
//
//// --- �ùķ��̼� (Simulation) ���� �Լ� ---
//void simulation_run(Simulation* sim); // ���� �ùķ��̼� ���� ����
//
//// =============================================================================
//// --- 4. �ý���, UI �� �α� ���� ---
//// =============================================================================
///**
// * @brief Windows �ֿܼ��� ANSI �̽������� �ڵ�(�÷� ��)�� ��� �����ϰ� �մϴ�.
// */
//void system_enable_virtual_terminal() {
//#ifdef _WIN32 // �� �ڵ�� Windows������ �����ϵ˴ϴ�.
//    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE); // ǥ�� ��� �ڵ��� �����ɴϴ�.
//    if (hOut == INVALID_HANDLE_VALUE) return; // �ڵ� �������� ���� �� �Լ� ����
//    DWORD dwMode = 0; // �ܼ� ��带 ������ ����
//    if (!GetConsoleMode(hOut, &dwMode)) return; // ���� �ܼ� ��带 �����ɴϴ�. ���� �� ����
//    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING; // ���� �͹̳� ó�� ��带 Ȱ��ȭ�մϴ�.
//    SetConsoleMode(hOut, dwMode); // ����� �ܼ� ��带 �����մϴ�.
//#endif
//}
//
///**
// * @brief ȭ���� ����ϴ�. Windows������ Ŀ�� ��ġ�� �� ���� �Ű� �������� ���Դϴ�.
// */
//void ui_clear_screen_optimized() {
//#ifdef _WIN32 // Windows ȯ���� ���
//    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); // ǥ�� ��� �ڵ��� �����ɴϴ�.
//    COORD coordScreen = { 0, 0 }; // ȭ���� �� �� ���� ��ǥ(0, 0)
//    SetConsoleCursorPosition(hConsole, coordScreen); // �ܼ� Ŀ���� �ش� ��ǥ�� �̵���ŵ�ϴ�.
//#else // Linux, macOS �� �ٸ� ȯ���� ���
//    printf("\033[H"); // ANSI �̽������� �ڵ带 ����Ͽ� Ŀ���� Ȩ ��ġ�� �̵���ŵ�ϴ�.
//#endif
//}
//
///**
//* @brief Enter Ű ���� ���� ���ڸ� ��� �Է¹޽��ϴ�. (Windows/Linux ȣȯ)
//* @return �Էµ� ����
//*/
//static char get_single_char() {
//#ifdef _WIN32 // Windows ȯ��
//    return _getch(); // _getch() �Լ� ���
//#else // Linux/macOS ȯ��
//    char buf = 0; // ���� ����
//    struct termios old = { 0 }; // �͹̳� ������ ������ ����ü
//    fflush(stdout); // ��� ���� ����
//    if (tcgetattr(0, &old) < 0) // ���� �͹̳� ���� ��������
//        perror("tcsetattr()");
//    old.c_lflag &= ~ICANON; // ���� �Է� ���(line-buffered) ��Ȱ��ȭ
//    old.c_lflag &= ~ECHO;   // �Է� ���� ȭ�鿡 ǥ��(echo) ��Ȱ��ȭ
//    old.c_cc[VMIN] = 1;     // �ּ� 1���� ���ڸ� ���� ������ ���
//    old.c_cc[VTIME] = 0;    // Ÿ�Ӿƿ� ����
//    if (tcsetattr(0, TCSANOW, &old) < 0) // ����� �͹̳� ���� ��� ����
//        perror("tcsetattr icanon");
//    if (read(0, &buf, 1) < 0) // ǥ�� �Է¿��� 1����Ʈ �б�
//        perror("read()");
//    old.c_lflag |= ICANON;  // �͹̳� ������ ������� ���� (���� ��� Ȱ��ȭ)
//    old.c_lflag |= ECHO;    // ���� Ȱ��ȭ
//    if (tcsetattr(0, TCSADRAIN, &old) < 0) // ��� �Ϸ� �� �͹̳� ���� ����
//        perror("tcsetattr ~icanon");
//    return buf; // ���� ���� ��ȯ
//#endif
//}
//
///**
//* @brief ����ڷκ��� ��ȿ�� ���� �� �ϳ��� �Է¹޽��ϴ�.
//* @param prompt ����ڿ��� ������ �ȳ� �޽���
//* @param valid_chars ��ȿ�� ���ڵ�� �̷���� ���ڿ� (��: "abq")
//* @return ����ڰ� �Է��� ��ȿ�� ����
//*/
//static char get_char_input(const char* prompt, const char* valid_chars) {
//    char choice;
//    while (TRUE) { // ��ȿ�� �Է��� ���� ������ �ݺ�
//        printf("%s", prompt); // �ȳ� �޽��� ���
//        choice = tolower(get_single_char()); // ���� ���ڸ� �Է¹޾� �ҹ��ڷ� ��ȯ
//        printf("%c\n", choice); // ����ڿ��� �Է� �ǵ�� ����
//        if (strchr(valid_chars, choice)) { // �Էµ� ���ڰ� ��ȿ�� ���� ��Ͽ� �ִ��� Ȯ��
//            return choice; // ��ȿ�ϸ� ���� ��ȯ
//        }
//        // ��ȿ���� ������ ���� �޽��� ���
//        printf(C_B_RED "\n�߸��� �Է��Դϴ�. ��ȿ�� Ű�� �����ּ���. (%s)\n" C_NRM, valid_chars);
//    }
//}
//
///**
// * @brief ����ڷκ��� Ư�� ���� ���� ������ �Է¹޽��ϴ�.
// */
//static int get_integer_input(const char* prompt, int min, int max) {
//    char buffer[INPUT_BUFFER_SIZE]; // �Է� ����
//    int value;
//    while (TRUE) { // ��ȿ�� �Է��� ���� ������ �ݺ�
//        printf("%s", prompt); // �ȳ� �޽��� ���
//        if (fgets(buffer, sizeof(buffer), stdin) != NULL) { // �� ���� �Է¹���
//            // �Է¹��� ���ڿ��� ������ ��ȯ�ϰ�, ���� ���� �ִ��� Ȯ��
//            if (sscanf(buffer, "%d", &value) == 1 && value >= min && value <= max) return value;
//        }
//        // ��ȿ���� ������ ���� �޽��� ���
//        printf(C_B_RED "�߸��� �Է��Դϴ�. %d���� %d ������ ������ �Է��ϼ���.\n" C_NRM, min, max);
//    }
//}
//
///**
// * @brief ����ڷκ��� Ư�� ���� ���� �Ǽ��� �Է¹޽��ϴ�.
// */
//static float get_float_input(const char* prompt, float min, float max) {
//    char buffer[INPUT_BUFFER_SIZE]; // �Է� ����
//    float value;
//    while (TRUE) { // ��ȿ�� �Է��� ���� ������ �ݺ�
//        printf("%s", prompt); // �ȳ� �޽��� ���
//        if (fgets(buffer, sizeof(buffer), stdin) != NULL) { // �� ���� �Է¹���
//            // �Է¹��� ���ڿ��� �Ǽ��� ��ȯ�ϰ�, ���� ���� �ִ��� Ȯ��
//            if (sscanf(buffer, "%f", &value) == 1 && value >= min && value <= max) return value;
//        }
//        // ��ȿ���� ������ ���� �޽��� ���
//        printf(C_B_RED "�߸��� �Է��Դϴ�. %.1f���� %.1f ������ ���ڸ� �Է��ϼ���.\n" C_NRM, min, max);
//    }
//}
//
///**
// * @brief ����� ���� �ó������� �����մϴ�.
// */
//static int simulation_setup_custom_scenario(ScenarioManager* scenario) {
//    printf(C_B_WHT "--- ����� ���� �ó����� ���� ---\n" C_NRM);
//    scenario->num_phases = get_integer_input(C_YEL "�� �ܰ� ���� �Է��ϼ��� (1-20, 0=���): " C_NRM, 0, MAX_PHASES);
//    if (scenario->num_phases == 0) return 0; // 0�� �Է��ϸ� ���
//
//    for (int i = 0; i < scenario->num_phases; i++) { // �� �ܰ迡 ���� ����
//        printf(C_B_CYN "\n--- %d/%d �ܰ� ���� ---\n" C_NRM, i + 1, scenario->num_phases);
//        printf("a. %s����%s\n", C_YEL, C_NRM);
//        printf("b. %s����%s\n", C_CYN, C_NRM);
//        char type_choice = get_char_input("�ܰ� ������ �����ϼ���: ", "ab");
//        scenario->phases[i].task_count = get_integer_input("�� �ܰ迡�� ó���� ���� ���� �Է��ϼ���: ", 1, 100);
//
//        if (type_choice == 'a') { // 'a'�� �����ϸ� ���� �ܰ�
//            scenario->phases[i].type = PARK_PHASE;
//            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "����");
//        }
//        else { // 'b'�� �����ϸ� ���� �ܰ�
//            scenario->phases[i].type = EXIT_PHASE;
//            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "����");
//        }
//        printf(C_GRN "%d�ܰ� ���� �Ϸ�: %s %d��.\n" C_NRM, i + 1, scenario->phases[i].type_name, scenario->phases[i].task_count);
//    }
//    printf(C_B_GRN "\n--- �ó����� ������ �Ϸ�Ǿ����ϴ�! ---\n" C_NRM);
//    sleep(1500); // 1.5�� ���
//    return 1; // ����
//}
//
///**
// * @brief �ǽð� �ùķ��̼��� �Ķ���͸� �����մϴ�.
// */
//static int simulation_setup_realtime(ScenarioManager* scenario) {
//    printf(C_B_WHT "--- �ǽð� �ùķ��̼� ���� ---\n" C_NRM);
//    while (TRUE) { // ��ȿ�� Ȯ������ �Էµ� ������ �ݺ�
//        scenario->park_chance = get_integer_input("\n���� ��û �߻� Ȯ��(0~100)�� �Է��ϼ���: ", 0, 100);
//        scenario->exit_chance = get_integer_input("���� ��û �߻� Ȯ��(0~100)�� �Է��ϼ���: ", 0, 100);
//        if ((scenario->park_chance + scenario->exit_chance) <= 100) break; // �� Ȯ���� ���� 100 ���ϸ� ���� Ż��
//        printf(C_B_RED "������ ���� Ȯ���� ���� 100�� ���� �� �����ϴ�.\n" C_NRM);
//    }
//    printf(C_B_GRN "\n���� �Ϸ�: ���� Ȯ�� %d%%, ���� Ȯ�� %d%%\n" C_NRM, scenario->park_chance, scenario->exit_chance);
//    sleep(1500); // 1.5�� ���
//    return 1; // ����
//}
//
///**
// * @brief �ùķ��̼� �ӵ��� �����մϴ�.
// */
//static int simulation_setup_speed(ScenarioManager* scenario) {
//    printf(C_B_WHT "\n--- �ùķ��̼� �ӵ� ���� ---\n" C_NRM);
//    scenario->speed_multiplier = get_float_input("���ϴ� ����� �Է��ϼ��� (1.0 ~ 100.0): ", 1.0f, MAX_SPEED_MULTIPLIER);
//    // ��ӿ� �ݺ���Ͽ� ���� �ð� ����
//    scenario->simulation_speed = (int)(100.0f / scenario->speed_multiplier);
//    if (scenario->simulation_speed < 1) scenario->simulation_speed = 1; // �ּ� ���� �ð��� 1ms
//    printf(C_B_GRN "\n--- �ùķ��̼��� %.1fx ������� �����մϴ�... ---\n" C_NRM, scenario->speed_multiplier);
//    sleep(1500); // 1.5�� ���
//    return 1; // ����
//}
//
///**
// * @brief �ùķ��̼� ���� �� ��ü ������ ����մϴ�.
// */
//int simulation_setup(Simulation* sim) {
//    ui_clear_screen_optimized(); // ȭ�� �����
//    printf(C_B_WHT "--- �ùķ��̼� ��� ���� ---\n" C_NRM);
//    printf("a. %s����� ���� �ó�����%s\n", C_YEL, C_NRM);
//    printf("b. %s�ǽð� �ùķ��̼�%s\n", C_CYN, C_NRM);
//    printf("q. %s����%s\n\n", C_RED, C_NRM);
//
//    char choice = get_char_input("������ �ó����� ���ڸ� �Է��ϼ���: ", "abq"); // ����ڷκ��� ��� ����
//    int setup_success = 0;
//    switch (choice) {
//    case 'a': // ����� ���� �ó�����
//        sim->scenario_manager->mode = MODE_CUSTOM;
//        if (simulation_setup_custom_scenario(sim->scenario_manager)) { // �ó����� ������ �����ϸ�
//            setup_success = simulation_setup_speed(sim->scenario_manager); // �ӵ� ����
//        }
//        break;
//    case 'b': // �ǽð� �ùķ��̼�
//        sim->scenario_manager->mode = MODE_REALTIME;
//        if (simulation_setup_realtime(sim->scenario_manager)) { // �ǽð� ������ �����ϸ�
//            setup_success = simulation_setup_speed(sim->scenario_manager); // �ӵ� ����
//        }
//        break;
//    case 'q': // ����
//        return 0; // ����(����) ��ȯ
//    }
//
//    if (setup_success) { // ������ ���������� �Ϸ�Ǹ�
//        ui_clear_screen_optimized(); // �ùķ��̼� ���� �� ȭ���� �����ϰ� ����
//    }
//    return setup_success; // ���� ���� ���� ��ȯ
//}
//
///**
//* @brief �׸��� ���� ���� ���¸� ���ڿ� ���ۿ� �������մϴ�.
//* @param buffer �������� ���ڿ��� ������ ����
//* @param buffer_size ������ ��ü ũ��
//* @param map GridMap ��ü
//* @param agent_manager AgentManager ��ü
//* @return ���ۿ� ������ ����Ʈ ��
//*/
//static int grid_map_render_to_buffer(char* buffer, size_t buffer_size, const GridMap* map, const AgentManager* agent_manager) {
//    char view[GRID_HEIGHT][GRID_WIDTH]; // ȭ�鿡 ǥ�õ� ���ڸ� ������ 2D �迭
//    const char* colors[GRID_HEIGHT][GRID_WIDTH]; // �� ������ ������ ������ 2D �迭
//    char* buf_ptr = buffer; // ������ ���� ��ġ�� ����Ű�� ������
//    size_t remaining_size = buffer_size; // ������ ���� ũ��
//    int written = 0; // �� ���� ������ ����Ʈ ��
//
//    // 1. ��� ���� ��Ʈ���� �ʱ�ȭ (�⺻ Ÿ��)
//    for (int y = 0; y < GRID_HEIGHT; y++) {
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            if (map->grid[y][x].is_obstacle) { // ��ֹ��� ���
//                view[y][x] = '+'; colors[y][x] = C_WHT;
//            }
//            else { // ���� ���
//                view[y][x] = '.'; colors[y][x] = C_GRY;
//            }
//        }
//    }
//
//    // 2. Ư�� ����(������, ���� ����) �׸���
//    for (int i = 0; i < map->num_charge_stations; i++) { // ��� �����ҿ� ����
//        Node* cs = map->charge_stations[i];
//        view[cs->y][cs->x] = 'e'; // 'e'�� ǥ��
//        int is_charging_here = FALSE; // �ش� ��ġ���� ���� ������ Ȯ���ϴ� �÷���
//        for (int j = 0; j < MAX_AGENTS; j++) {
//            if (agent_manager->agents[j].state == CHARGING && agent_manager->agents[j].pos == cs) {
//                is_charging_here = TRUE; break;
//            }
//        }
//        colors[cs->y][cs->x] = is_charging_here ? C_B_RED : C_B_YEL; // ���� ���̸� ������, �ƴϸ� �����
//    }
//    for (int i = 0; i < map->num_goals; i++) { // ��� ���� ������ ����
//        Node* g = map->goals[i];
//        if (g->is_parked) { // ������ ���
//            view[g->y][g->x] = 'P'; colors[g->y][g->x] = C_RED;
//        }
//        else if (g->is_goal) { // ����ִ� ���� ������ ���
//            view[g->y][g->x] = 'G'; colors[g->y][g->x] = C_GRN;
//        }
//    }
//
//    // 3. ������Ʈ �׸��� (�ٸ� ��ҵ��� ������� �������� �׸�)
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        if (agent_manager->agents[i].pos) { // ������Ʈ ��ġ�� ��ȿ�ϸ�
//            view[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = agent_manager->agents[i].symbol;
//            if (i == 0) colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_CYN;
//            else if (i == 1) colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_YEL;
//            else colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_MAG;
//        }
//    }
//
//    // 4. �ϼ��� ��� ���� ��Ʈ������ ������� ���ۿ� ���ڿ� ����
//    written = snprintf(buf_ptr, remaining_size, C_B_WHT "\n--- LPA* Parking Simulation (v11.0 LL) ---\n" C_NRM);
//    buf_ptr += written; remaining_size -= written;
//    for (int y = 0; y < GRID_HEIGHT; y++) {
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            written = snprintf(buf_ptr, remaining_size, "%s %c" C_NRM, colors[y][x], view[y][x]);
//            buf_ptr += written; remaining_size -= written;
//        }
//        written = snprintf(buf_ptr, remaining_size, "\n");
//        buf_ptr += written; remaining_size -= written;
//    }
//    written = snprintf(buf_ptr, remaining_size, "\n");
//    buf_ptr += written; remaining_size -= written;
//    return buf_ptr - buffer; // �� ������ ����Ʈ �� ��ȯ
//}
//
///**
//* @brief �ùķ��̼��� ��� ���� ������ ���ۿ� �� ��, �� ���� ȭ�鿡 ����մϴ�.
//* @param sim Simulation ��ü
//*/
//static void simulation_display_status(const Simulation* sim) {
//    const ScenarioManager* scenario = sim->scenario_manager;
//    const AgentManager* agent_manager = sim->agent_manager;
//    const GridMap* map = sim->map;
//    const Logger* logger = sim->logger;
//    char display_buffer[DISPLAY_BUFFER_SIZE]; // ȭ�� ��ü�� ���� ū ����
//    char* buf_ptr = display_buffer; // ���� ������
//    size_t remaining_size = sizeof(display_buffer); // ���� ���� ũ��
//    int written = 0; // ������ ����Ʈ ��
//
//    // --- 1. ���� ��� ���� ���ۿ� ���� ---
//    written = snprintf(buf_ptr, remaining_size, C_B_WHT);
//    buf_ptr += written; remaining_size -= written;
//
//    if (scenario->mode == MODE_CUSTOM) { // ����� ���� ����� ��
//        if (scenario->current_phase_index < scenario->num_phases) {
//            DynamicPhase* p = &scenario->phases[scenario->current_phase_index];
//            written = snprintf(buf_ptr, remaining_size, "--- Custom Scenario: %d/%d [Speed: %.1fx] ---\n", scenario->current_phase_index + 1, scenario->num_phases, scenario->speed_multiplier);
//            buf_ptr += written; remaining_size -= written;
//            written = snprintf(buf_ptr, remaining_size, "Time: %d, Current Task: %s (%d/%d)\n", scenario->time_step, p->type_name, scenario->tasks_completed_in_phase, p->task_count);
//            buf_ptr += written; remaining_size -= written;
//        }
//        else { // ��� �ܰ� �Ϸ�
//            written = snprintf(buf_ptr, remaining_size, "--- Custom Scenario: All phases complete ---\n");
//            buf_ptr += written; remaining_size -= written;
//        }
//    }
//    else if (scenario->mode == MODE_REALTIME) { // �ǽð� ����� ��
//        written = snprintf(buf_ptr, remaining_size, "--- Real-Time Simulation [Speed: %.1fx] ---\n", scenario->speed_multiplier);
//        buf_ptr += written; remaining_size -= written;
//
//        int park_requests = 0;
//        int exit_requests = 0;
//        TaskNode* current_task = scenario->task_queue_head;
//        while (current_task != NULL) {
//            if (current_task->type == TASK_PARK) {
//                park_requests++;
//            }
//            else if (current_task->type == TASK_EXIT) {
//                exit_requests++;
//            }
//            current_task = current_task->next;
//        }
//
//        written = snprintf(buf_ptr, remaining_size, "Time: %d / %d | Pending Tasks: %d (%sPark Req: %d%s, %sExit Req: %d%s)\n",
//            scenario->time_step,
//            REALTIME_MODE_TIMELIMIT,
//            scenario->task_count,
//            C_B_GRN, park_requests, C_NRM, // ���� ��û�� ���� �ʷϻ�
//            C_B_YEL, exit_requests, C_NRM  // ���� ��û�� ���� �����
//        );
//        buf_ptr += written; remaining_size -= written;
//    }
//    written = snprintf(buf_ptr, remaining_size, "Parked Cars: %d/%d\n" C_NRM, agent_manager->total_cars_parked, map->num_goals);
//    buf_ptr += written; remaining_size -= written;
//
//    // --- 2. �׸��� �� ���� ���ۿ� ���� ---
//    written = grid_map_render_to_buffer(buf_ptr, remaining_size, map, agent_manager);
//    buf_ptr += written; remaining_size -= written;
//
//    // --- 3. ������Ʈ ���� ���� ���ۿ� ���� ---
//    const char* agent_state_strings[] = { "���", "���� ��", "���� ����(�� ��)", "���� ��", "���� ��(���� ž��)", "�����ҷ� �̵�", "���� ��", "���� ����(���� ��)" };
//    const char* agent_state_colors[] = { C_GRY, C_YEL, C_CYN, C_YEL, C_GRN, C_B_RED, C_RED, C_CYN };
//
//
//    for (int i = 0; i < MAX_AGENTS; i++) { // ��� ������Ʈ�� ����
//        const Agent* agent = &agent_manager->agents[i];
//        const char* agent_color = (i == 0) ? C_B_CYN : (i == 1) ? C_B_YEL : C_B_MAG;
//        char status_buffer[100]; // ���� ���ڿ��� �ӽ÷� ������ ����
//
//        if (agent->state == CHARGING) { // ���� ���� ��� Ÿ�̸� ǥ��
//            snprintf(status_buffer, sizeof(status_buffer), "���� ��... (%d)", agent->charge_timer);
//        }
//        else { // �� �� ����
//            snprintf(status_buffer, sizeof(status_buffer), "%s", agent_state_strings[agent->state]);
//        }
//
//        // ������Ʈ ID�� ���� ��ġ (x,y)
//        written = snprintf(buf_ptr, remaining_size, "%sAgent %c%s: (%2d,%d) ", agent_color, agent->symbol, C_NRM, agent->pos->x, agent->pos->y);
//        buf_ptr += written; remaining_size -= written;
//
//        if (agent->goal) { // ��ǥ�� ���� ��� ��ǥ ��ġ
//            written = snprintf(buf_ptr, remaining_size, "-> (%2d,%d) ", agent->goal->x, agent->goal->y);
//        }
//        else { // ��ǥ�� ���� ���
//            written = snprintf(buf_ptr, remaining_size, "-> ����       ");
//        }
//        buf_ptr += written; remaining_size -= written;
//
//        // �̵� �Ÿ�(���ϸ���)�� ���� ���ڿ�
//        written = snprintf(buf_ptr, remaining_size, "[Mileage: %6.1f/%d] [%s%-*s%s]\n",
//            agent->total_distance_traveled, (int)DISTANCE_BEFORE_CHARGE,
//            agent_state_colors[agent->state], STATUS_STRING_WIDTH, status_buffer, C_NRM);
//        buf_ptr += written; remaining_size -= written;
//    }
//    written = snprintf(buf_ptr, remaining_size, "\n");
//    buf_ptr += written; remaining_size -= written;
//
//    // --- 4. �α� ���� ���ۿ� ���� ---
//    written = snprintf(buf_ptr, remaining_size, C_B_WHT "--- Simulation Log ---\n" C_NRM);
//    buf_ptr += written; remaining_size -= written;
//    for (int i = 0; i < logger->log_count; i++) {
//        int index = (logger->log_head + i) % LOG_BUFFER_LINES; // ��ȯ ���� �ε��� ���
//        written = snprintf(buf_ptr, remaining_size, "%s%s%s\n", C_GRY, logger->logs[index], C_NRM);
//        buf_ptr += written; remaining_size -= written;
//    }
//
//    // --- 5. �ϼ��� ���۸� ȭ�鿡 �� ���� ��� ---
//    ui_clear_screen_optimized(); // ȭ���� �����
//    printf("%s", display_buffer); // ������ ������ �� ���� ��� (������ �ּ�ȭ)
//}
//
//// --- Logger ---
///**
// * @brief Logger ��ü�� �����ϰ� �޸𸮸� �Ҵ��մϴ�.
// */
//Logger* logger_create() {
//    Logger* logger = (Logger*)calloc(1, sizeof(Logger)); // Logger ����ü ũ�⸸ŭ �޸𸮸� �Ҵ��ϰ� 0���� �ʱ�ȭ
//    if (!logger) { // �޸� �Ҵ� ���� ��
//        perror("Logger �Ҵ� ����"); // ���� �޽��� ���
//        exit(1); // ���α׷� ����
//    }
//    return logger; // ������ Logger ��ü ������ ��ȯ
//}
//
///**
// * @brief Logger ��ü�� �޸𸮸� �����մϴ�.
// */
//void logger_destroy(Logger* logger) {
//    if (logger) free(logger); // logger�� NULL�� �ƴϸ� �޸� ����
//}
//
///**
// * @brief ��ȯ �α� ���ۿ� �α� �޽����� �߰��մϴ�.
// */
//void logger_log(Logger* logger, const char* format, ...) {
//    va_list args; // ���� ���� ���
//    va_start(args, format); // ���� ���� ��� �ʱ�ȭ
//    // �α׸� ������ ���� ��ġ ��� (��ȯ ����)
//    int current_log_index = (logger->log_head + logger->log_count) % LOG_BUFFER_LINES;
//    // ���˿� ���� �α� �޽����� ���ۿ� ��
//    vsnprintf(logger->logs[current_log_index], LOG_BUFFER_WIDTH, format, args);
//    va_end(args); // ���� ���� ��� ����
//
//    if (logger->log_count < LOG_BUFFER_LINES) { // ���۰� ���� ���� ���� �ʾҴٸ�
//        logger->log_count++; // �α� ī��Ʈ ����
//    }
//    else { // ���۰� ���� á�ٸ�
//        logger->log_head = (logger->log_head + 1) % LOG_BUFFER_LINES; // ���� ������ �α׸� ������� head�� �̵�
//    }
//}
//
//// =============================================================================
//// --- 5. ��ƿ��Ƽ ���� (�켱���� ť) ---
//// =============================================================================
//// --- Priority Queue (min-heap) ---
///**
// * @brief �� ���� Key ����ü�� ���մϴ�. k1�� ����, ������ k2�� ���մϴ�.
// * @return k1 < k2 �̸� -1, k1 > k2 �̸� 1, ������ 0�� ��ȯ�մϴ�.
// */
//static int compare_keys(Key k1, Key k2) {
//    if (k1.k1 < k2.k1 - 1e-9) return -1; // �ε��Ҽ��� ������ ����Ͽ� k1.k1�� ������ ��
//    if (k1.k1 > k2.k1 + 1e-9) return 1;  // �ε��Ҽ��� ������ ����Ͽ� k1.k1�� ū�� ��
//    if (k1.k2 < k2.k2 - 1e-9) return -1; // k1.k1�� ���� ���, k1.k2�� ������ ��
//    if (k1.k2 > k2.k2 + 1e-9) return 1;  // k1.k1�� ���� ���, k1.k2�� ū�� ��
//    return 0; // �� Ű�� ������
//}
//
///**
// * @brief �켱���� ť�� �ʱ�ȭ�մϴ�.
// */
//void pq_init(PriorityQueue* pq, int capacity) {
//    pq->nodes = (Node**)malloc(sizeof(Node*) * capacity); // ��� ������ �迭�� ���� �޸� �Ҵ�
//    pq->size = 0; // ���� ũ�⸦ 0���� ����
//    pq->capacity = capacity; // �ִ� �뷮 ����
//}
//
///**
// * @brief �켱���� ť�� �޸𸮸� �����մϴ�.
// */
//void pq_free(PriorityQueue* pq) {
//    if (pq && pq->nodes) { // ť�� ��� �迭�� ��ȿ�� ���
//        free(pq->nodes); // ��� ������ �迭 �޸� ����
//        pq->nodes = NULL; // ��۸� ������ ����
//    }
//}
//
///**
// * @brief �� ���� �� ����� ��ġ�� �ٲߴϴ�.
// */
//static void swap_nodes(Node** a, Node** b) {
//    Node* t = *a; *a = *b; *b = t; // �����͸� �̿��� ����
//    int temp_idx = (*a)->pq_index;
//    (*a)->pq_index = (*b)->pq_index;
//    (*b)->pq_index = temp_idx; // �� ��尡 ���� pq_index ���� ����
//}
//
///**
// * @brief ���� Ư�� ��ġ���� ���� �ö󰡸� �� �Ӽ��� ������Ű���� �������մϴ�. (heapify up)
// */
//static void heapify_up(PriorityQueue* pq, int i) {
//    if (i == 0) return; // ��Ʈ ����̸� ����
//    int p = (i - 1) / 2; // �θ� ����� �ε��� ���
//    if (compare_keys(pq->nodes[i]->key, pq->nodes[p]->key) < 0) { // �ڽ� ����� Ű ���� �θ𺸴� ������
//        swap_nodes(&pq->nodes[i], &pq->nodes[p]); // �ڽİ� �θ� ����
//        heapify_up(pq, p); // �θ� ��ġ���� �ٽ� heapify_up ��� ȣ��
//    }
//}
//
///**
// * @brief ���� Ư�� ��ġ���� �Ʒ��� �������� �� �Ӽ��� ������Ű���� �������մϴ�. (heapify down)
// */
//static void heapify_down(PriorityQueue* pq, int i) {
//    int l = 2 * i + 1, r = 2 * i + 2, s = i; // l: ���� �ڽ�, r: ������ �ڽ�, s: ���� ���� ���� ���� ����� �ε���
//    if (l < pq->size && compare_keys(pq->nodes[l]->key, pq->nodes[s]->key) < 0) s = l; // ���� �ڽ��� �� ������ s�� l��
//    if (r < pq->size && compare_keys(pq->nodes[r]->key, pq->nodes[s]->key) < 0) s = r; // ������ �ڽ��� �� ������ s�� r��
//    if (s != i) { // ���� ���(i)�� ���� ���� ���� �ƴϸ�
//        swap_nodes(&pq->nodes[i], &pq->nodes[s]); // ���� ���� ���� ���� �ڽİ� ����
//        heapify_down(pq, s); // ���ҵ� ��ġ���� �ٽ� heapify_down ��� ȣ��
//    }
//}
//
///**
// * @brief �켱���� ť�� ��带 �߰��մϴ�.
// */
//void pq_push(PriorityQueue* pq, Node* n) {
//    if (pq->size >= pq->capacity) return; // ť�� ���� á���� ����
//    n->in_pq = TRUE; // ��尡 ť�� ������ ǥ��
//    n->pq_index = pq->size; // ����� �� �ε����� ���� ť�� ������ ��ġ�� ����
//    pq->nodes[pq->size++] = n; // �迭�� �������� ��带 �߰��ϰ� ������ ����
//    heapify_up(pq, pq->size - 1); // �߰��� ��ġ���� heapify_up ����
//}
//
///**
// * @brief �켱���� ť���� ���� �켱������ ����(Ű ���� ����) ��带 �����ϰ� ��ȯ�մϴ�.
// */
//Node* pq_pop(PriorityQueue* pq) {
//    if (pq->size == 0) return NULL; // ť�� ��������� NULL ��ȯ
//    Node* top = pq->nodes[0]; // ��Ʈ ���(���� ���� ��)�� top�� ����
//    top->in_pq = FALSE; // ť���� ���ŵǾ����� ǥ��
//    top->pq_index = -1; // ť �ε����� -1�� ����
//    pq->size--; // ť ������ ����
//    if (pq->size > 0) { // ť�� ��尡 ����������
//        pq->nodes[0] = pq->nodes[pq->size]; // ���� ������ ��带 ��Ʈ�� �̵�
//        pq->nodes[0]->pq_index = 0; // ��Ʈ ����� �ε����� 0���� ����
//        heapify_down(pq, 0); // ��Ʈ�������� heapify_down ����
//    }
//    return top; // �����ص� top ��� ��ȯ
//}
//
///**
// * @brief �켱���� ť���� Ư�� ��带 �����մϴ�.
// */
//void pq_remove(PriorityQueue* pq, Node* n) {
//    if (!n->in_pq) return; // ��尡 ť�� ������ ����
//    int idx = n->pq_index; // ������ ����� �ε���
//    pq->size--; // ť ������ ����
//    if (idx != pq->size) { // ������ ��尡 ������ ��尡 �ƴϾ��ٸ�
//        pq->nodes[idx] = pq->nodes[pq->size]; // ������ ��带 ������ ����� ��ġ�� �̵�
//        pq->nodes[idx]->pq_index = idx; // �ε��� ������Ʈ
//        // �̵��� ����� Ű ���� ���� heapify_up �Ǵ� heapify_down ����
//        if (idx > 0 && compare_keys(pq->nodes[idx]->key, pq->nodes[(idx - 1) / 2]->key) < 0) {
//            heapify_up(pq, idx);
//        }
//        else {
//            heapify_down(pq, idx);
//        }
//    }
//    n->in_pq = FALSE; // ť���� ���ŵǾ����� ǥ��
//    n->pq_index = -1; // ť �ε��� ����
//}
//
///**
// * @brief ��尡 �켱���� ť�� ���ԵǾ� �ִ��� Ȯ���մϴ�.
// */
//int pq_contains(const Node* n) { return n->in_pq; } // ����� in_pq �÷��� ��ȯ
//
///**
// * @brief �켱���� ť�� �ֻ���(���� ����) Ű ���� ��ȯ�մϴ�. (��带 �������� ����)
// */
//Key pq_top_key(const PriorityQueue* pq) {
//    if (pq->size == 0) return (Key) { INF, INF }; // ť�� ��������� ���Ѵ� Ű �� ��ȯ
//    return pq->nodes[0]->key; // ��Ʈ ����� Ű �� ��ȯ
//}
//
//
//// =============================================================================
//// --- 6. �� ���� ���� (GridMap) ---
//// =============================================================================
///**
// * @brief �ڵ� ���� �ϵ��ڵ��� �� �����ͷκ��� ���� �ε��մϴ�.
// */
//static void grid_map_load_from_embedded_map(GridMap* map, AgentManager* agent_manager) {
//    // 2D ������ �� ������
//    const char* embedded_map_data =
//        "1111111111111111111111111111111111111\n" // y=0
//        "C01GGG1GG1GGG1GGG1GGG1GGG1GGG1G11G111\n" // y=1
//        "A000000000000000000000000000000000001\n" // y=2
//        "B0000000000000000000000000000000000001\n" // y=3
//        "1111GG1GG1GGG10001GGG1GGG1GGG1100e111\n" // y=4
//        "111111111111110001GGG1GGG1GGG11001111\n" // y=5
//        "100000000000000000000000000000000e111\n" // y=6
//        "100000000000000000000000000000000e111\n" // y=7
//        "11100001111111GGG1GGG1GGG1GGG1GG11111\n" // y=8
//        "1000000000000000000000000000000000001\n" // y=9
//        "1GG1GG1GG1GG1GG1GG1GG1GG1GG1GG1GG1GG1\n" // y=10
//        "1111111111111111111111111111111111111\n";
//
//
//    map->num_goals = 0; // ��ǥ(���� ����) �� �ʱ�ȭ
//    map->num_charge_stations = 0; // ������ �� �ʱ�ȭ
//    int map_idx = 0; // �� ������ ���ڿ��� ���� �ε���
//    for (int y = 0; y < GRID_HEIGHT; y++) { // ���� ���̸�ŭ �ݺ�
//        for (int x = 0; x < GRID_WIDTH; x++) { // ���� �ʺ�ŭ �ݺ�
//            char ch;
//            do { ch = embedded_map_data[map_idx++]; } while (ch == '\n' || ch == '\r'); // ���� ���ڴ� �ǳʶ�
//            if (ch == '\0') { // �� �����Ͱ� ���󺸴� ª���� ���� ó��
//                fprintf(stderr, C_B_RED "����: ����� �� �����Ͱ� ���󺸴� ª���ϴ�.\n" C_NRM);
//                exit(1);
//            }
//
//            Node* n = &map->grid[y][x]; // ���� (x, y) ��ǥ�� ��忡 ���� ������
//            // ��� �⺻������ �ʱ�ȭ
//            *n = (Node){ .x = x, .y = y, .g = INF, .rhs = INF, .is_obstacle = FALSE,
//            .is_goal = FALSE, .is_temp = FALSE, .is_parked = FALSE,
//            .reserved_by_agent = -1, .in_pq = FALSE, .pq_index = -1 };
//
//            switch (ch) { // �о�� ���ڿ� ���� ��� �Ӽ� ����
//            case '1': n->is_obstacle = TRUE; break; // '1'�� ��ֹ�
//            case 'A': agent_manager->agents[0].pos = n; agent_manager->agents[0].home_base = n; break; // 'A'�� ������Ʈ 0�� ���� ��ġ
//            case 'B': agent_manager->agents[1].pos = n; agent_manager->agents[1].home_base = n; break; // 'B'�� ������Ʈ 1�� ���� ��ġ
//            case 'C': agent_manager->agents[2].pos = n; agent_manager->agents[2].home_base = n; break; // 'C'�� ������Ʈ 2�� ���� ��ġ
//            case 'G': n->is_goal = TRUE; if (map->num_goals < MAX_GOALS) map->goals[map->num_goals++] = n; break; // 'G'�� ���� ���� (��ǥ)
//            case 'e': if (map->num_charge_stations < MAX_CHARGE_STATIONS) map->charge_stations[map->num_charge_stations++] = n; break; // 'e'�� ������
//            }
//        }
//    }
//    if (map->num_charge_stations == 0) { // �ʿ� �����Ұ� �ϳ��� ������ ���� ó��
//        fprintf(stderr, C_B_RED "����: �� �����Ϳ� ������('e')�� �����ϴ�.\n" C_NRM);
//        exit(1);
//    }
//}
//
///**
// * @brief GridMap ��ü�� �����ϰ� �ʱ�ȭ�մϴ�.
// */
//GridMap* grid_map_create(AgentManager* agent_manager) {
//    GridMap* map = (GridMap*)calloc(1, sizeof(GridMap)); // GridMap ����ü �޸� �Ҵ� �� 0���� �ʱ�ȭ
//    if (!map) { // �Ҵ� ���� ��
//        perror("GridMap �Ҵ� ����");
//        exit(1);
//    }
//    grid_map_load_from_embedded_map(map, agent_manager); // ����� �� ������ �ε�
//    return map; // ������ map ��ü ������ ��ȯ
//}
//
///**
// * @brief GridMap ��ü�� �޸𸮸� �����մϴ�.
// */
//void grid_map_destroy(GridMap* map) {
//    if (map) free(map); // map�� NULL�� �ƴϸ� �޸� ����
//}
//
///**
// * @brief �־��� ��ǥ�� �׸��� ���� ���� �ִ��� Ȯ���մϴ�.
// */
//int grid_is_valid_coord(int x, int y) {
//    return x >= 0 && x < GRID_WIDTH&& y >= 0 && y < GRID_HEIGHT; // x, y�� 0 �̻��̰� �ִ� �ʺ�/���� �̸����� Ȯ��
//}
//
///**
// * @brief �־��� ��尡 ���� �̵� �Ұ����� �������� Ȯ���մϴ�.
// */
//int grid_is_node_blocked(const GridMap* map, const AgentManager* agent_manager, const Node* n) {
//    if (n->is_obstacle || n->is_parked || n->is_temp) { // ���� ��ֹ�, ������ ��, �ӽ� ��ֹ��̸�
//        return TRUE; // ����
//    }
//    for (int i = 0; i < MAX_AGENTS; i++) { // ��� ������Ʈ�� ����
//        // �ٸ� ������Ʈ�� �ش� ��ġ���� ���� ���̶��
//        if (agent_manager->agents[i].pos == n && agent_manager->agents[i].state == CHARGING) {
//            return TRUE; // ����
//        }
//    }
//    return FALSE; // �� �ܿ��� ������ ����
//}
//
//// =============================================================================
//// --- 7. �ó����� �� �۾� ���� ���� (ScenarioManager) ---
//// =============================================================================
///**
// * @brief ScenarioManager�� �۾� ť�� ���� ��� ����� �޸𸮸� �����մϴ�.
// */
//static void scenario_manager_clear_task_queue(ScenarioManager* manager) {
//    TaskNode* current = manager->task_queue_head;
//    TaskNode* next_node;
//    while (current != NULL) {
//        next_node = current->next;
//        free(current);
//        current = next_node;
//    }
//    manager->task_queue_head = NULL;
//    manager->task_queue_tail = NULL;
//    manager->task_count = 0;
//}
//
///**
// * @brief ScenarioManager ��ü�� �����ϰ� �ʱ�ȭ�մϴ�.
// */
//ScenarioManager* scenario_manager_create() {
//    ScenarioManager* manager = (ScenarioManager*)calloc(1, sizeof(ScenarioManager)); // �޸� �Ҵ� �� 0���� �ʱ�ȭ
//    if (!manager) { // �Ҵ� ���� ��
//        perror("ScenarioManager �Ҵ� ����");
//        exit(1);
//    }
//    manager->simulation_speed = 100; // �⺻ �ùķ��̼� �ӵ� (���� �ð�)
//    manager->speed_multiplier = 1.0f; // �⺻ ���
//    manager->park_chance = 40; // �⺻ ���� ��û Ȯ��
//    manager->exit_chance = 30; // �⺻ ���� ��û Ȯ��
//    // ���� ����Ʈ ť �ʱ�ȭ
//    manager->task_queue_head = NULL;
//    manager->task_queue_tail = NULL;
//    manager->task_count = 0;
//    return manager; // ������ manager ��ü ������ ��ȯ
//}
//
///**
// * @brief ScenarioManager ��ü�� �޸𸮸� �����մϴ�.
// */
//void scenario_manager_destroy(ScenarioManager* manager) {
//    if (manager) {
//        scenario_manager_clear_task_queue(manager); // ť�� ���� �۾� �޸� ����
//        free(manager); // manager�� NULL�� �ƴϸ� �޸� ����
//    }
//}
//
///**
// * @brief �۾� ť(���� ����Ʈ)�� ���ο� �۾��� �߰��մϴ�. (�ǽð� ����)
// */
//static void add_task_to_queue(ScenarioManager* scenario, TaskType type) {
//    if (scenario->task_count >= MAX_TASKS) return; // ť�� ���� á���� �߰����� ����
//
//    TaskNode* new_task = (TaskNode*)malloc(sizeof(TaskNode));
//    if (!new_task) {
//        perror("Failed to allocate memory for new task");
//        return;
//    }
//    new_task->type = type;
//    new_task->next = NULL;
//
//    if (scenario->task_queue_head == NULL) { // ť�� ������� ���
//        scenario->task_queue_head = new_task;
//        scenario->task_queue_tail = new_task;
//    }
//    else { // ť�� ��尡 ���� ���
//        scenario->task_queue_tail->next = new_task;
//        scenario->task_queue_tail = new_task;
//    }
//    scenario->task_count++;
//}
//
//// =============================================================================
//// --- 8. ��� Ž�� ���� (Pathfinder) ---
//// =============================================================================
///**
// * @brief �� ��� ���� ����ư �Ÿ��� ����ϴ� �޸���ƽ �Լ�.
// */
//static double heuristic(const Node* a, const Node* b) {
//    return fabs(a->x - b->x) + fabs(a->y - b->y); // |x1 - x2| + |y1 - y2|
//}
//
///**
// * @brief �� ��� ���� ��Ŭ���� �Ÿ��� ����մϴ�.
// */
//static double euclidean_distance(const Node* a, const Node* b) {
//    return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2)); // sqrt((x1-x2)^2 + (y1-y2)^2)
//}
//
///**
// * @brief LPA* �˰��� ���� ����� �켱���� Ű�� ����մϴ�.
// */
//static Key calculate_key(const Pathfinder* pf, const Node* n) {
//    double m = fmin(n->g, n->rhs); // g���� rhs�� �� ���� ���� ����
//    return (Key) { m + heuristic(n, pf->start_node), m }; // k1 = min(g, rhs) + h, k2 = min(g, rhs)
//}
//
///**
// * @brief ����� rhs ���� ������Ʈ�ϰ�, �ʿ� �� �켱���� ť�� �߰�/������Ʈ�մϴ�.
// */
//static void path_update_vertex(Pathfinder* pf, GridMap* map, const AgentManager* agent_manager, Node* u) {
//    if (u != pf->goal_node) { // ��ǥ ��尡 �ƴ϶��
//        double min_rhs = INF; // �ּ� rhs ���� ���Ѵ�� �ʱ�ȭ
//        int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 }; // �����¿� �̵�
//        for (int i = 0; i < NUM_DIRECTIONS; i++) { // 4���⿡ ����
//            int nx = u->x + dx[i], ny = u->y + dy[i]; // �̿� ��� ��ǥ ���
//            if (grid_is_valid_coord(nx, ny)) { // �̿��� �׸��� ���� �ִٸ�
//                Node* successor = &map->grid[ny][nx]; // �̿� ��� ������
//                if (!grid_is_node_blocked(map, agent_manager, successor)) { // �̿��� �������� �ʴٸ�
//                    min_rhs = fmin(min_rhs, successor->g + 1.0); // rhs ���� ���� (g(s') + c(s', s))
//                }
//            }
//        }
//        u->rhs = min_rhs; // ���� �ּҰ����� rhs ������Ʈ
//    }
//
//    if (pq_contains(u)) { // ��尡 �̹� ť�� �ִٸ�
//        pq_remove(&pf->pq, u); // �ϴ� ����
//    }
//
//    // g�� rhs�� ��ġ���� ������ (inconsistent)
//    if (fabs(u->g - u->rhs) > 1e-9) {
//        u->key = calculate_key(pf, u); // Ű ���� �ٽ� ����ϰ�
//        pq_push(&pf->pq, u); // ť�� �ٽ� �߰� (�켱������ ���ŵ�)
//    }
//}
//
///**
// * @brief ��� Ž����(Pathfinder)�� �����մϴ�.
// */
//Pathfinder* pathfinder_create(Node* start, Node* goal) {
//    Pathfinder* pf = (Pathfinder*)malloc(sizeof(Pathfinder)); // �޸� �Ҵ�
//    if (!pf) return NULL; // �Ҵ� ���� �� NULL ��ȯ
//    pf->start_node = start; // ���� ��� ����
//    pf->goal_node = goal; // ��ǥ ��� ����
//    pq_init(&pf->pq, GRID_WIDTH * GRID_HEIGHT); // �켱���� ť �ʱ�ȭ
//    return pf; // ������ ��� Ž���� ��ȯ
//}
//
///**
// * @brief ��� Ž����(Pathfinder)�� �޸𸮸� �����մϴ�.
// */
//void pathfinder_destroy(Pathfinder* pf) {
//    if (pf) { // ��� Ž���Ⱑ ��ȿ�ϴٸ�
//        pq_free(&pf->pq); // �켱���� ť �޸� ����
//        free(pf); // ��� Ž���� ��ü �޸� ����
//    }
//}
//
///**
// * @brief LPA* �˰����� ���� ������ �����Ͽ� �ִ� ��θ� ����մϴ�.
// */
//void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, AgentManager* agent_manager) {
//    // �׸��� �� ��� ����� g, rhs ���� �ʱ�ȭ�մϴ�.
//    for (int y = 0; y < GRID_HEIGHT; y++) {
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            Node* n = &map->grid[y][x];
//            n->g = INF; n->rhs = INF; n->in_pq = FALSE; n->pq_index = -1;
//        }
//    }
//
//    if (pf->goal_node) { // ��ǥ ��尡 �����Ǿ� �ִٸ�
//        pf->goal_node->rhs = 0; // ��ǥ ����� rhs�� 0���� ����
//        pf->goal_node->key = calculate_key(pf, pf->goal_node); // Ű ���� ���
//        pq_push(&pf->pq, pf->goal_node); // �켱���� ť�� ��ǥ ��带 �߰�
//    }
//
//    int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 }; // �����¿� �̵�
//
//    // ť�� ������� �ʰ�, ���� ����� g, rhs�� ��ġ���� �ʰų�, ť�� top key�� ���� ����� key���� ���� ���� �ݺ�
//    while (pf->pq.size > 0 &&
//        (compare_keys(pq_top_key(&pf->pq), calculate_key(pf, pf->start_node)) < 0 ||
//            fabs(pf->start_node->rhs - pf->start_node->g) > 1e-9)) {
//
//        Node* u = pq_pop(&pf->pq); // ť���� �켱������ ���� ���� ��带 ����
//        if (u->g > u->rhs) { // overconsistent ������ �� (�� ���� ��� �߰�)
//            u->g = u->rhs; // g���� rhs������ ������Ʈ (consistent ���·� ����)
//            for (int i = 0; i < NUM_DIRECTIONS; i++) { // ��� �̿�(predecessor)�� ����
//                int px = u->x + dx[i], py = u->y + dy[i];
//                if (grid_is_valid_coord(px, py)) path_update_vertex(pf, map, agent_manager, &map->grid[py][px]); // �̿� ��� ������Ʈ
//            }
//        }
//        else { // underconsistent ������ �� (���� ��ο� ���� �߻�)
//            u->g = INF; // g���� ���Ѵ�� ����
//            path_update_vertex(pf, map, agent_manager, u); // �ڱ� �ڽ��� ���� ������Ʈ
//            for (int i = 0; i < NUM_DIRECTIONS; i++) { // ��� �̿��� ����
//                int px = u->x + dx[i], py = u->y + dy[i];
//                if (grid_is_valid_coord(px, py)) path_update_vertex(pf, map, agent_manager, &map->grid[py][px]); // �̿� ��� ������Ʈ
//            }
//        }
//    }
//}
//
///**
// * @brief ���� ���(g��)�� �������� ���� ��ġ���� �̵��� ���� ��带 �����մϴ�.
// */
//Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager, Node* current_node) {
//    // ��ǥ�� ���ų�, ���� ��忡�� ��ǥ������ ��ΰ� ���ų�, �̹� ��ǥ�� �����ߴٸ�
//    if (!pf->goal_node || current_node->g >= INF || current_node == pf->goal_node) {
//        return current_node; // ���� ��ġ�� �ӹ���
//    }
//
//    double min_g = INF; // �̿� �� ���� ���� g���� ������ ����
//    Node* best_next_node = current_node; // ������ ���� ���, �⺻���� ���� ���
//    double min_dist_to_goal = euclidean_distance(current_node, pf->goal_node); // ��ǥ������ ���� �Ÿ�
//
//    int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 }; // �����¿� �̵�
//
//    for (int i = 0; i < NUM_DIRECTIONS; i++) { // 4������ �̿��� ����
//        int nx = current_node->x + dx[i], ny = current_node->y + dy[i];
//        if (grid_is_valid_coord(nx, ny)) { // ��ȿ�� ��ǥ���
//            Node* neighbor = &map->grid[ny][nx]; // �̿� ��� ������
//            if (grid_is_node_blocked(map, agent_manager, neighbor)) continue; // ���� ���̸� �ǳʶ�
//
//            if (neighbor->g < min_g) { // �̿��� g���� ���� �ּ� g������ ������ (�� ���� ���)
//                min_g = neighbor->g; // �ּ� g�� ����
//                best_next_node = neighbor; // ������ ���� ��带 �� �̿����� ����
//                min_dist_to_goal = euclidean_distance(neighbor, pf->goal_node); // ��ǥ������ �Ÿ��� ����
//            }
//            // g���� ���� ���, ��ǥ������ ��Ŭ���� �Ÿ��� �� ª�� ���� ���� (tie-breaking)
//            else if (fabs(neighbor->g - min_g) < 1e-9) {
//                double dist_to_goal = euclidean_distance(neighbor, pf->goal_node);
//                if (dist_to_goal < min_dist_to_goal) {
//                    best_next_node = neighbor;
//                    min_dist_to_goal = dist_to_goal;
//                }
//            }
//        }
//    }
//    return best_next_node; // ������ ������ ���� ��� ��ȯ
//}
//
//
//// =============================================================================
//// --- 9. ������Ʈ ���� �� ���� ���� (AgentManager) ---
//// =============================================================================
///**
// * @brief AgentManager ��ü�� �����ϰ� �ʱ�ȭ�մϴ�.
// */
//AgentManager* agent_manager_create() {
//    AgentManager* manager = (AgentManager*)calloc(1, sizeof(AgentManager)); // �޸� �Ҵ� �� 0���� �ʱ�ȭ
//    if (!manager) { // �Ҵ� ���� ��
//        perror("AgentManager �Ҵ� ����");
//        exit(1);
//    }
//    for (int i = 0; i < MAX_AGENTS; i++) { // �� ������Ʈ �ʱ�ȭ
//        manager->agents[i].id = i; // ID ����
//        manager->agents[i].symbol = 'A' + i; // �ɺ� ���� ('A', 'B', 'C')
//        manager->agents[i].state = IDLE; // �ʱ� ���´� '���'
//    }
//    return manager; // ������ manager ��ü ������ ��ȯ
//}
//
///**
// * @brief AgentManager ��ü�� �޸𸮸� �����մϴ�.
// */
//void agent_manager_destroy(AgentManager* manager) {
//    if (manager) free(manager); // manager�� NULL�� �ƴϸ� �޸� ����
//}
//
///**
// * @brief Ư�� ������Ʈ�� Ư�� ��ǥ������ ��� ���(g��)�� ����մϴ�.
// */
//static double calculate_path_cost(Agent* agent, Node* goal, GridMap* map, AgentManager* agent_manager) {
//    Pathfinder* pf = pathfinder_create(agent->pos, goal); // ��� Ž���� ����
//    if (!pf) return INF; // ���� ���� �� ���Ѵ� ��� ��ȯ
//    pathfinder_compute_shortest_path(pf, map, agent_manager); // ��� ���
//    double cost = agent->pos->g; // ���� ��ġ�� g���� �ٷ� ���
//    pathfinder_destroy(pf); // ��� Ž���� �޸� ����
//    return cost; // ���� ��� ��ȯ
//}
//
///**
// * @brief ���� ������Ʈ���� ���� ȿ������(�����) ���� ������ �����մϴ�.
// */
//static Node* select_best_parking_spot(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
//    double best_cost = INF; // ���� ����� ���Ѵ�� �ʱ�ȭ
//    Node* best_goal = NULL; // ���� ��ǥ�� NULL�� �ʱ�ȭ
//    for (int j = 0; j < map->num_goals; j++) { // ��� ���� ������ ����
//        Node* g = map->goals[j]; // ���� ���� ��� ������
//        // �̹� �����Ǿ� �ְų� �ٸ� ������Ʈ�� �����ߴٸ� �ǳʶ�
//        if (g->is_parked || (g->reserved_by_agent != -1 && g->reserved_by_agent != agent->id)) continue;
//
//        double cost = calculate_path_cost(agent, g, map, agent_manager); // �ش� ���� ���������� ��� ���
//        if (cost < best_cost) { // ��������� ���� ��뺸�� �����ϸ�
//            best_cost = cost; // ���� ��� ����
//            best_goal = g; // ���� ��ǥ ����
//        }
//    }
//    if (best_goal) { // ���� ��ǥ�� ã�Ҵٸ�
//        logger_log(logger, "[%sPlan%s] Agent %c, ���� ���� (%d,%d) ���� (���: %.1f)", C_CYN, C_NRM, agent->symbol, best_goal->x, best_goal->y, best_cost);
//    }
//    return best_goal; // ã�� ������ ���� ���� ��ȯ
//}
//
///**
// * @brief ���� ������Ʈ���� ���� ȿ������(�����) ������ ������ �����մϴ�.
// */
//static Node* select_best_parked_car(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
//    double best_cost = INF; // ���� ����� ���Ѵ�� �ʱ�ȭ
//    Node* best_spot = NULL; // ���� ��ǥ�� NULL�� �ʱ�ȭ
//    for (int j = 0; j < map->num_goals; j++) { // ��� ���� ������ ����
//        Node* g = map->goals[j]; // ���� ���� ��� ������
//        // ������ ���� ���ų� �ٸ� ������Ʈ�� �����ߴٸ� �ǳʶ�
//        if (!g->is_parked || (g->reserved_by_agent != -1 && g->reserved_by_agent != agent->id)) continue;
//
//        g->is_parked = FALSE; // ��� ����� ���� '������' ���¸� �ӽ÷� ����
//        double cost = calculate_path_cost(agent, g, map, agent_manager); // ��� ���
//        g->is_parked = TRUE;  // ���� ���·� ����
//
//        if (cost < best_cost) { // �� ������ ����� ã����
//            best_cost = cost; // ���� ��� ����
//            best_spot = g; // ���� ��ǥ ����
//        }
//    }
//    if (best_spot) { // ���� ��ǥ�� ã�Ҵٸ�
//        logger_log(logger, "[%sPlan%s] Agent %c, ���� ���� (%d,%d) ���� (���: %.1f)", C_CYN, C_NRM, agent->symbol, best_spot->x, best_spot->y, best_cost);
//    }
//    return best_spot; // ã�� ������ ���� ���� ��ġ ��ȯ
//}
//
///**
// * @brief ���� ������Ʈ���� ���� ȿ������(�����) �����Ҹ� �����մϴ�.
// */
//static Node* select_best_charge_station(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
//    double best_cost = INF; // ���� ��� �ʱ�ȭ
//    Node* best_station = NULL; // ���� ������ �ʱ�ȭ
//    for (int i = 0; i < map->num_charge_stations; i++) { // ��� �����ҿ� ����
//        Node* station = map->charge_stations[i]; // ������ ��� ������
//        // �ٸ� ������Ʈ�� �����ߴٸ� �ǳʶ�
//        if (station->reserved_by_agent != -1 && station->reserved_by_agent != agent->id) continue;
//        double cost = calculate_path_cost(agent, station, map, agent_manager); // ��� ���
//        if (cost < best_cost) { // �� ������ ����� ã����
//            best_cost = cost; // ��� ����
//            best_station = station; // ������ ����
//        }
//    }
//    if (best_station) { // ���� �����Ҹ� ã�Ҵٸ�
//        logger_log(logger, "[%sPlan%s] Agent %c, ������ (%d,%d) ���� (���: %.1f)", C_CYN, C_NRM, agent->symbol, best_station->x, best_station->y, best_cost);
//    }
//    return best_station; // ã�� ������ ������ ��ȯ
//}
//
///**
// * @brief ������Ʈ�� ���� ���¿� ���� ��ǥ(goal)�� �����մϴ�.
// */
//static void agent_set_goal(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
//    // '�� ���� ���� ��'�� ���� ������ �ʿ��ϸ� ��� ��θ� �����մϴ�.
//    // ������ ��� ��(�����Ϸ� ���ų�, ������Ű�� ���� ��)�� ���� ���� �۾��� �Ϸ��ؾ� �ϹǷ�, �� ��쿡�� ��θ� �������� �ʽ��ϴ�.
//    if (agent->state == RETURNING_HOME_EMPTY && agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
//        if (agent->goal) { // �̹� ������ ���ϴ� ��ǥ�� �����Ǿ� �־��ٸ�
//            agent->goal->reserved_by_agent = -1; // ���� ��ǥ�� ������ ����
//            agent->goal = NULL; // ��ǥ�� �ʱ�ȭ�Ͽ� ���ο� ��ǥ�� ������ �� �ֵ��� ��
//        }
//        logger_log(logger, "[%sCharge%s] Agent %c ���� �ʿ�! ��ǥ�� �����ҷ� ��� ����.", C_B_YEL, C_NRM, agent->symbol);
//        agent->state = GOING_TO_CHARGE; // ���¸� �����ҷ� ���� ���·� ����
//    }
//
//    // ��� ���̰ų�, ���� ���̰ų�, �̹� ��ǥ�� ������ ��쿣 ���ο� ��ǥ�� �������� ����
//    if (agent->state == IDLE || agent->state == CHARGING || agent->goal != NULL) return;
//
//    switch (agent->state) { // ������Ʈ ���¿� ����
//    case GOING_TO_PARK: agent->goal = select_best_parking_spot(agent, map, agent_manager, logger); break; // ���� ���� ���� ����
//    case RETURNING_HOME_EMPTY:
//    case RETURNING_WITH_CAR:
//    case RETURNING_HOME_MAINTENANCE:
//        agent->goal = agent->home_base; break; // ��ǥ�� ������ ����
//    case GOING_TO_COLLECT: agent->goal = select_best_parked_car(agent, map, agent_manager, logger); break; // ���� ���� ���� ����
//    case GOING_TO_CHARGE: agent->goal = select_best_charge_station(agent, map, agent_manager, logger); break; // ���� ������ ����
//    default: break;
//    }
//
//    if (agent->goal) { // ��ǥ�� ���������� �����Ǿ��ٸ�
//        agent->goal->reserved_by_agent = agent->id; // �ش� ��ǥ�� ���� ������Ʈ�� �����ߴٰ� ǥ��
//    }
//    else if (agent->state != RETURNING_HOME_EMPTY && agent->state != RETURNING_WITH_CAR && agent->state != RETURNING_HOME_MAINTENANCE) { // ��ǥ ������ �����߰�, ���� ���� �ƴ϶��
//        agent->state = IDLE; // ��� ���·� ��ȯ
//        logger_log(logger, "[%sInfo%s] Agent %c: ���� ��ǥ ����. ��� ���·� ��ȯ.", C_YEL, C_NRM, agent->symbol);
//    }
//}
//
///**
// * @brief ��� ������Ʈ�� ���� �̵��� ��ȹ�ϰ�, �߻��� �� �ִ� �浹�� �ذ��մϴ�.
// */
//void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
//    // --- 1�ܰ�: ��ǥ ���� ---
//    // ��� ������Ʈ�� �ӽ� ��ֹ��� ���� '������' ���� �������� �̻����� ��ǥ�� �����մϴ�.
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        // ������Ʈ�� �� ��ǥ�� ���� �غ� �� ��쿡�� ��ǥ�� �����մϴ�. (��: ���� ��ǥ�� ���� ���)
//        if (agent->goal == NULL && agent->state != IDLE && agent->state != CHARGING) {
//            agent_set_goal(agent, map, manager, logger);
//        }
//    }
//
//    // ���� ��ġ�� ���� ��ġ�� �ʱ�ȭ
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        next_pos[i] = manager->agents[i].pos;
//    }
//
//    // --- 2�ܰ�: ��� ��ȹ �� �浹 ȸ�� ---
//    // �� ������Ʈ�� ���õ� ��ǥ�� ���� ���� �� ������ ��ȹ�ϸ�, �� �������� �ٸ� ������Ʈ�� ���մϴ�.
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        if (agent->state == IDLE || agent->state == CHARGING || agent->goal == NULL) {
//            continue; // ��Ȱ�� �����̰ų� ��ǥ�� ���� ������Ʈ�� �ǳʶݴϴ�.
//        }
//
//        // �� ������Ʈ�� ��� Ž���� ���� �ٸ� ������Ʈ���� �ӽ� ��ֹ��� ����
//        Node* obstacles_to_clear[MAX_AGENTS];
//        int obs_count = 0;
//        for (int j = 0; j < MAX_AGENTS; j++) {
//            if (i == j) continue;
//            // �켱������ ����(���� ����) ������Ʈ�� �̹� ���� ���� ��ġ��, �ƴ� ���� ���� ��ġ�� ��ֹ��� ����
//            Node* obs_node = (j < i) ? next_pos[j] : manager->agents[j].pos;
//            if (obs_node) {
//                obs_node->is_temp = TRUE;
//                obstacles_to_clear[obs_count++] = obs_node;
//            }
//        }
//
//        // ��ǥ�� �缱������ �ʰ� ���� ���ܸ� ��ȹ
//        int goal_was_parked = (agent->state == GOING_TO_COLLECT && agent->goal->is_parked);
//        if (goal_was_parked) agent->goal->is_parked = FALSE;
//
//        Pathfinder* pf = pathfinder_create(agent->pos, agent->goal);
//        pathfinder_compute_shortest_path(pf, map, manager);
//        next_pos[i] = pathfinder_get_next_step(pf, map, manager, agent->pos);
//        pathfinder_destroy(pf);
//
//        if (goal_was_parked) agent->goal->is_parked = TRUE;
//
//        // �ӽ� ��ֹ� ���� ����
//        for (int k = 0; k < obs_count; k++) {
//            if (obstacles_to_clear[k]) obstacles_to_clear[k]->is_temp = FALSE;
//        }
//    }
//
//    // --- 3�ܰ�: �浹 �ذ� ---
//    // ���� ���� ���ܵ��� ������� ���� �浹�� �ذ��մϴ�.
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        for (int j = i + 1; j < MAX_AGENTS; j++) {
//            if (manager->agents[i].state == IDLE || manager->agents[j].state == IDLE ||
//                manager->agents[i].state == CHARGING || manager->agents[j].state == CHARGING) continue;
//
//            // �浹 ���� 1: �� ������Ʈ�� ���� ���� ��ġ�� �̵��Ϸ��� ���
//            if (next_pos[i] == next_pos[j]) {
//                logger_log(logger, "[%sAvoid%s] �浹 ����! Agent %c ���.", C_B_RED, C_NRM, manager->agents[j].symbol);
//                next_pos[j] = manager->agents[j].pos; // �켱������ ���� j�� ���� ��ġ�� ���
//            }
//            // �浹 ���� 2: �� ������Ʈ�� ������ ��ġ�� �¹ٲٷ� �ϴ� ��� (���� �浹)
//            else if (next_pos[i] == manager->agents[j].pos && next_pos[j] == manager->agents[i].pos) {
//                logger_log(logger, "[%sAvoid%s] ���� �浹 ����! Agent %c ���.", C_B_RED, C_NRM, manager->agents[j].symbol);
//                next_pos[j] = manager->agents[j].pos; // �켱������ ���� j�� ���� ��ġ�� ���
//            }
//        }
//    }
//}
//
///**
// * @brief ������Ʈ�� �̵��� ��ģ �� ���¸� ������Ʈ�մϴ�. (��ǥ ���� ó��)
// */
//void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger) {
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        // ��Ȱ�� �����̰ų�, ��ǥ�� ���ų�, ���� ��ǥ�� �������� ���ߴٸ� �ǳʶ�
//        if (agent->state == IDLE || agent->state == CHARGING || !agent->goal || agent->pos != agent->goal) {
//            continue;
//        }
//
//        Node* reached_goal = agent->goal; // ������ ��ǥ ���
//
//        if (agent->state != GOING_TO_CHARGE) {
//            reached_goal->reserved_by_agent = -1;
//        }
//        agent->goal = NULL; // ������Ʈ�� ��ǥ �ʱ�ȭ
//
//        switch (agent->state) {
//        case GOING_TO_PARK: // �����Ϸ� ���� ���̾��ٸ�
//            reached_goal->is_parked = TRUE;
//            manager->total_cars_parked++;
//            logger_log(logger, "[%sPark%s] Agent %c, ���� �Ϸ� at (%d,%d).", C_GRN, C_NRM, agent->symbol, reached_goal->x, reached_goal->y);
//
//            // [Task Count Fix] ���� �Ϸ� ������ �ٷ� �۾� ī��Ʈ�� ������ŵ�ϴ�.
//            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == PARK_PHASE) {
//                scenario->tasks_completed_in_phase++;
//            }
//
//            agent->state = RETURNING_HOME_EMPTY;
//            break;
//
//        case RETURNING_HOME_EMPTY: // ���� �� ������ ���� ���̾��ٸ�
//            logger_log(logger, "[%sInfo%s] Agent %c, ���� �۾� �� ���� ���� �Ϸ�.", C_CYN, C_NRM, agent->symbol);
//            // �۾� ī��Ʈ�� ���� ������ �̹� �Ϸ�Ǿ����Ƿ� ���⼭�� �ƹ��͵� ���� �ʽ��ϴ�.
//            agent->state = IDLE;
//            break;
//
//        case GOING_TO_COLLECT:
//            logger_log(logger, "[%sExit%s] Agent %c, ���� ���� at (%d,%d).", C_YEL, C_NRM, agent->symbol, reached_goal->x, reached_goal->y);
//            reached_goal->is_parked = FALSE;
//            manager->total_cars_parked--;
//            agent->state = RETURNING_WITH_CAR;
//            break;
//
//        case RETURNING_WITH_CAR: // ���� �ư� ���� ���̾��ٸ�
//            logger_log(logger, "[%sExit%s] Agent %c, ���� ���� �Ϸ�.", C_GRN, C_NRM, agent->symbol);
//
//            // [Task Count Confirmed] ���� �ܰ迴�ٸ�, ���� ���� ������ �۾� �Ϸ� ī��Ʈ�� �ø��ϴ�.
//            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == EXIT_PHASE) {
//                scenario->tasks_completed_in_phase++;
//            }
//            agent->state = IDLE;
//            break;
//
//        case GOING_TO_CHARGE:
//            logger_log(logger, "[%sCharge%s] Agent %c, ���� ����. (%d steps)", C_B_YEL, C_NRM, agent->symbol, CHARGE_TIME);
//            agent->state = CHARGING;
//            agent->charge_timer = CHARGE_TIME;
//            break;
//
//        case RETURNING_HOME_MAINTENANCE:
//            logger_log(logger, "[%sInfo%s] Agent %c, ���� �� ���� ���� �Ϸ�.", C_CYN, C_NRM, agent->symbol);
//            agent->state = IDLE;
//            break;
//
//        default: break;
//        }
//    }
//}
//
///**
// * @brief ���� ���� ������Ʈ�� ���¸� ������Ʈ�մϴ�.
// */
//void agent_manager_update_charge_state(AgentManager* manager, Logger* logger) {
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        if (agent->state == CHARGING) { // ���� ������ ������Ʈ�� ����
//            agent->charge_timer--; // ���� Ÿ�̸� ����
//            if (agent->charge_timer <= 0) { // Ÿ�̸Ӱ� 0 ���ϰ� �Ǹ� ���� �Ϸ�
//                logger_log(logger, "[%sCharge%s] Agent %c ���� �Ϸ�.", C_B_GRN, C_NRM, agent->symbol);
//                agent->total_distance_traveled = 0.0; // �̵� �Ÿ� �ʱ�ȭ
//                agent->state = RETURNING_HOME_MAINTENANCE;
//                if (agent->pos) agent->pos->reserved_by_agent = -1; // ���� ��ġ(������)�� ���� ����
//                agent->goal = NULL; // ��ǥ �ʱ�ȭ
//            }
//        }
//    }
//}
//
//// =============================================================================
//// --- 10. �ùķ��̼� �ھ� ���� ���� ---
//// =============================================================================
///**
// * @brief ��ü �ùķ��̼� ��ü(Simulation)�� �����ϰ� ���� ������ �ʱ�ȭ�մϴ�.
// */
//Simulation* simulation_create() {
//    Simulation* sim = (Simulation*)calloc(1, sizeof(Simulation)); // Simulation ����ü �޸� �Ҵ� �� 0���� �ʱ�ȭ
//    if (!sim) { perror("Simulation �Ҵ� ����"); exit(1); } // �Ҵ� ���� �� ���� ó��
//
//    sim->agent_manager = agent_manager_create(); // ������Ʈ �Ŵ��� ����
//    sim->map = grid_map_create(sim->agent_manager); // �� ���� (������Ʈ �ʱ� ��ġ ������ ���� agent_manager �ʿ�)
//    sim->scenario_manager = scenario_manager_create(); // �ó����� �Ŵ��� ����
//    sim->logger = logger_create(); // �ΰ� ����
//    return sim; // ������ �ùķ��̼� ��ü ������ ��ȯ
//}
//
///**
// * @brief �ùķ��̼� ��ü�� ��� ���� ����� �޸𸮸� �����մϴ�.
// */
//void simulation_destroy(Simulation* sim) {
//    if (sim) { // �ùķ��̼� ��ü�� ��ȿ�ϴٸ�
//        grid_map_destroy(sim->map); // �� �޸� ����
//        agent_manager_destroy(sim->agent_manager); // ������Ʈ �Ŵ��� �޸� ����
//        scenario_manager_destroy(sim->scenario_manager); // �ó����� �Ŵ��� �޸� ����
//        logger_destroy(sim->logger); // �ΰ� �޸� ����
//        free(sim); // �ùķ��̼� ��ü ��ü �޸� ����
//    }
//}
//
///**
// * @brief �ùķ��̼��� ���¸� �� ���� ������Ʈ�մϴ�. (�۾� �Ҵ� ��)
// */
//static void simulation_update_state(Simulation* sim) {
//    ScenarioManager* scenario = sim->scenario_manager;
//    AgentManager* agent_manager = sim->agent_manager;
//    GridMap* map = sim->map;
//    Logger* logger = sim->logger;
//
//    // --- ����� ���� �ó����� ��� ���� ---
//    if (scenario->mode == MODE_CUSTOM) {
//        if (scenario->current_phase_index >= scenario->num_phases) return; // ��� �ܰ谡 �Ϸ�Ǿ����� ����
//
//        DynamicPhase* phase = &scenario->phases[scenario->current_phase_index]; // ���� �ܰ� ����
//        // ���� �ܰ迡�� �䱸�ϴ� �۾� ���� ��� �Ϸ�Ǿ��ٸ�
//        if (scenario->tasks_completed_in_phase >= phase->task_count) {
//            logger_log(logger, "[%sPhase%s] %d�ܰ� (%s %d��) �Ϸ�!", C_B_YEL, C_NRM, scenario->current_phase_index + 1, phase->type_name, phase->task_count);
//            scenario->current_phase_index++; // ���� �ܰ�� �̵�
//            scenario->tasks_completed_in_phase = 0; // �Ϸ�� �۾� �� �ʱ�ȭ
//            if (scenario->current_phase_index < scenario->num_phases) { // ���� �ܰ谡 �ִٸ�
//                DynamicPhase* next_phase = &scenario->phases[scenario->current_phase_index]; // ���� �ܰ� ����
//                logger_log(logger, "[%sPhase%s] %d�ܰ� ����: %s %d��.", C_B_YEL, C_NRM, scenario->current_phase_index + 1, next_phase->type_name, next_phase->task_count);
//                sleep(1500); // �ܰ� ��ȯ �� ��� ���
//            }
//            return;
//        }
//    }
//    // --- �ǽð� �ùķ��̼� ��� ���� ---
//    else if (scenario->mode == MODE_REALTIME) {
//        // ���� �ð� ���ݸ��� ���ο� �۾�(�̺�Ʈ)�� ����
//        if (scenario->time_step > 0 && scenario->time_step % EVENT_GENERATION_INTERVAL == 0) {
//            int event_chance = rand() % 100; // 0~99 ������ ���� ����
//            // ���� Ȯ���� �ش��ϰ�, ���� ������ ����������
//            if (event_chance < scenario->park_chance && agent_manager->total_cars_parked < map->num_goals) {
//                logger_log(logger, "[%sEvent%s] ���ο� ���� ��û �߻�.", C_B_GRN, C_NRM);
//                add_task_to_queue(scenario, TASK_PARK); // ���� �۾��� ť�� �߰�
//            }
//            // ���� Ȯ���� �ش��ϰ�, ������ ���� ������
//            else if (event_chance < (scenario->park_chance + scenario->exit_chance) && agent_manager->total_cars_parked > 0) {
//                logger_log(logger, "[%sEvent%s] ���ο� ���� ��û �߻�.", C_B_YEL, C_NRM);
//                add_task_to_queue(scenario, TASK_EXIT); // ���� �۾��� ť�� �߰�
//            }
//        }
//    }
//
//    // --- ��� ������Ʈ�� ���� �۾� �Ҵ� ���� ---
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &agent_manager->agents[i];
//        if (agent->state == IDLE) { // ��� ���� ������Ʈ���Ը� �۾� �Ҵ� �õ�
//            // ������ �ʿ����� ���� Ȯ��
//            if (agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
//                if (select_best_charge_station(agent, map, agent_manager, logger)) { // ������ �����Ұ� �ִٸ�
//                    agent->state = GOING_TO_CHARGE; // �����ҷ� ���� ���·� ����
//                }
//                else { // ��� �����Ұ� ��� ���̸�
//                    logger_log(logger, "[%sWarn%s] Agent %c ���� �ʿ��ϳ� ��� �����Ұ� ��� ��.", C_YEL, C_NRM, agent->symbol);
//                }
//                continue; // ������ �켱�̹Ƿ� �ٸ� �۾� �Ҵ��� �ǳʶ�
//            }
//
//            // [Ŀ���� ��� �۾� �Ҵ�]
//            if (scenario->mode == MODE_CUSTOM) {
//                if (scenario->current_phase_index >= scenario->num_phases) continue; // ��� �ܰ� �Ϸ� �� �ǳʶ�
//                DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
//
//                if (phase->type == PARK_PHASE) { // ���簡 ���� �ܰ��� ���
//                    int active_tasks = 0; // ���� ���� �۾��� ���� ���� ������Ʈ ��
//                    for (int j = 0; j < MAX_AGENTS; j++) {
//                        AgentState s = agent_manager->agents[j].state;
//                        if (s == GOING_TO_PARK || s == RETURNING_HOME_EMPTY) {
//                            active_tasks++;
//                        }
//                    }
//                    // (�Ϸ�� �۾� + ���� ���� �۾�)�� ��ǥ������ ����, ���� ������ ���Ҵٸ�
//                    if ((scenario->tasks_completed_in_phase + active_tasks) < phase->task_count && agent_manager->total_cars_parked < map->num_goals) {
//                        agent->state = GOING_TO_PARK; // ���� �۾� �Ҵ�
//                        logger_log(logger, "[%sTask%s] Agent %c, �ű� ���� �۾� �Ҵ�.", C_CYN, C_NRM, agent->symbol);
//                    }
//                }
//                else if (phase->type == EXIT_PHASE) { // ���簡 ���� �ܰ��� ���
//                    int active_tasks = 0; // ���� ���� �۾��� ���� ���� ������Ʈ ��
//                    for (int j = 0; j < MAX_AGENTS; j++) {
//                        AgentState s = agent_manager->agents[j].state;
//                        if (s == GOING_TO_COLLECT || s == RETURNING_WITH_CAR) {
//                            active_tasks++;
//                        }
//                    }
//                    // (�Ϸ�� �۾� + ���� ���� �۾�)�� ��ǥ������ ����, ������ ���� �ִٸ�
//                    if ((scenario->tasks_completed_in_phase + active_tasks) < phase->task_count && agent_manager->total_cars_parked > 0) {
//                        agent->state = GOING_TO_COLLECT; // ���� �۾� �Ҵ�
//                        logger_log(logger, "[%sTask%s] Agent %c, �ű� ���� �۾� �Ҵ�.", C_CYN, C_NRM, agent->symbol);
//                    }
//                }
//            }
//            // [�ǽð� ��� �۾� �Ҵ� - ���� ����Ʈ ���]
//            else if (scenario->mode == MODE_REALTIME && scenario->task_count > 0) {
//                int is_parking_lot_full = (agent_manager->total_cars_parked >= map->num_goals);
//                TaskNode* current = scenario->task_queue_head;
//                TaskNode* prev = NULL;
//                int task_assigned = FALSE;
//
//                while (current != NULL) {
//                    int can_process = FALSE;
//                    // �������� �� á�� ���: ���� ��û�� ã��
//                    if (is_parking_lot_full) {
//                        if (current->type == TASK_EXIT && agent_manager->total_cars_parked > 0) {
//                            can_process = TRUE;
//                        }
//                    }
//                    // �������� �� ���� �ʾ��� ���: ��� ��ȿ�� ��û�� ������� ó��
//                    else {
//                        if (current->type == TASK_PARK) {
//                            can_process = TRUE; // ���� ������ �����Ƿ� ���� ��û ó�� ����
//                        }
//                        else if (current->type == TASK_EXIT && agent_manager->total_cars_parked > 0) {
//                            can_process = TRUE; // ������ ���� �����Ƿ� ���� ��û ó�� ����
//                        }
//                    }
//
//                    if (can_process) {
//                        // �۾� �Ҵ�
//                        if (current->type == TASK_PARK) {
//                            agent->state = GOING_TO_PARK;
//                            logger_log(logger, "[%sTask%s] Agent %c, ���� �۾� �Ҵ�.", C_CYN, C_NRM, agent->symbol);
//                        }
//                        else { // TASK_EXIT
//                            agent->state = GOING_TO_COLLECT;
//                            logger_log(logger, "[%sTask%s] Agent %c, ���� �۾� �Ҵ�.", C_CYN, C_NRM, agent->symbol);
//                        }
//                        task_assigned = TRUE;
//
//                        // ť���� �ش� ��� ����
//                        if (prev == NULL) { // ������ ��尡 ����� ���
//                            scenario->task_queue_head = current->next;
//                        }
//                        else {
//                            prev->next = current->next;
//                        }
//                        if (current == scenario->task_queue_tail) { // ������ ��尡 ������ ���
//                            scenario->task_queue_tail = prev;
//                        }
//                        free(current);
//                        scenario->task_count--;
//                        break; // �� ������Ʈ�� ���� �۾� �Ҵ� �Ϸ�
//                    }
//
//                    // ���� ���� �̵�
//                    prev = current;
//                    current = current->next;
//                }
//            }
//        }
//    }
//}
//
//
///**
// * @brief �ùķ��̼��� �Ϸ�Ǿ����� Ȯ���մϴ�.
// */
//static int simulation_is_complete(const Simulation* sim) {
//    const ScenarioManager* scenario = sim->scenario_manager;
//    const AgentManager* agent_manager = sim->agent_manager;
//
//    // Ŀ���� ���: ��� �ܰ谡 ������, ��� ������Ʈ�� ��� ������ �� �Ϸ�
//    if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index >= scenario->num_phases) {
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            if (agent_manager->agents[i].state != IDLE) return FALSE; // ���� Ȱ�� ���� ������Ʈ�� ������ �̿Ϸ�
//        }
//        printf(C_B_GRN "\n��� �ó����� �ܰ谡 �Ϸ�Ǿ����ϴ�! �ùķ��̼��� �����մϴ�.\n" C_NRM);
//        return TRUE; // �Ϸ�
//    }
//
//    // �ǽð� ���: ������ �ð� ���ѿ� �������� �� �Ϸ�
//    if (scenario->mode == MODE_REALTIME && scenario->time_step >= REALTIME_MODE_TIMELIMIT) {
//        printf(C_B_GRN "\n�ǽð� �ùķ��̼� �ð� ���ѿ� �����߽��ϴ�! �ùķ��̼��� �����մϴ�.\n" C_NRM);
//        return TRUE; // �Ϸ�
//    }
//    return FALSE; // �̿Ϸ�
//}
//
///**
// * @brief ���� �ùķ��̼� ������ �����մϴ�.
// */
//void simulation_run(Simulation* sim) {
//    while (TRUE) { // ���� ����
//        // ���� ������Ʈ �� ���� ���
//        agent_manager_update_charge_state(sim->agent_manager, sim->logger); // 1. ���� ���� ������Ʈ
//        simulation_update_state(sim); // 2. �۾� �Ҵ� �� ��ü ���� ������Ʈ
//
//        Node* next_pos[MAX_AGENTS]; // �� ������Ʈ�� ���� ��ġ�� ������ �迭
//        agent_manager_plan_and_resolve_collisions(sim->agent_manager, sim->map, sim->logger, next_pos); // 3. ��� ��ȹ �� �浹 �ذ�
//
//        // ���� ���� ��ġ�� ������Ʈ �̵�
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            if (sim->agent_manager->agents[i].state != CHARGING && next_pos[i] != NULL) { // ���� ���� �ƴ� ���� �̵�
//                if (sim->agent_manager->agents[i].pos != next_pos[i]) { // ������ �̵��ߴٸ�
//                    sim->agent_manager->agents[i].total_distance_traveled += 1.0; // �̵� �Ÿ� ����
//                }
//                sim->agent_manager->agents[i].pos = next_pos[i]; // ��ġ ������Ʈ
//            }
//        }
//        agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->logger); // 4. �̵� �� ��ǥ ���� �� ���� ������Ʈ
//
//        // ����ȭ�� ������ �Լ� ȣ��
//        simulation_display_status(sim); // 5. ���� ���¸� ȭ�鿡 ǥ��
//
//        if (simulation_is_complete(sim)) break; // 6. �ùķ��̼� �Ϸ� ���� Ȯ�� �� ���� Ż��
//
//        sim->scenario_manager->time_step++; // �ð� ���� ����
//        sleep(sim->scenario_manager->simulation_speed); // ������ �ӵ���ŭ ���
//    }
//}
//
//
//// =============================================================================
//// --- 11. ���� �Լ� ---
//// =============================================================================
//int main() {
//    srand((unsigned int)time(NULL)); // ���� ������ �õ� ����
//    system_enable_virtual_terminal(); // Windows���� ANSI �÷� �ڵ� Ȱ��ȭ
//
//    Simulation* sim = simulation_create(); // �ùķ��̼� ��ü ����
//    if (!sim) return 1; // ���� ���� �� ����
//
//    if (simulation_setup(sim)) { // ����ڷκ��� �ùķ��̼� ������ �ް� �����ϸ�
//        simulation_run(sim); // �ùķ��̼� ���� ���� ����
//    }
//    else { // ������ ��ҵǸ�
//        printf("\n�ùķ��̼��� ��ҵǾ����ϴ�. ���α׷��� �����մϴ�.\n");
//    }
//
//    simulation_destroy(sim); // �Ҵ�� ��� �޸� ����
//    return 0; // ���α׷� ���� ����
//}
