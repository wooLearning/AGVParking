/**
* @file d_lite_parking.c
* @brief D* Lite �˰��� ��� 3-������Ʈ �ڵ� ���� �ùķ��̼� (ȸ�� ������ �� �۾� ������ ����)
*/
#define _CRT_SECURE_NO_WARNINGS // visual studio ȯ�濡�� ���� ��� ��Ȱ��ȭ�ϱ� ���� ��ũ��
#include <stdio.h>      // ǥ�� ����� �Լ�(printf, scanf ��)�� ����ϱ� ���� ��� ����
#include <stdlib.h>     // ���� �޸� �Ҵ�(malloc, free), ���� ����(rand) ���� ���� ��� ����
#include <string.h>     // ���ڿ� ó�� �Լ�(strcpy, strcmp ��)�� ����ϱ� ���� ��� ����
#include <math.h>       // ���� �Լ�(sqrt, fabs ��)�� ����ϱ� ���� ��� ����
#include <float.h>      // �ε��Ҽ��� ���� �ִ밪(DBL_MAX) �� �Ѱ谪�� ������ ��� ����
#include <time.h>       // �ð� ���� �Լ�(time)�� ����ϱ� ���� ��� ���� (���� �õ� ������ ���)
#include <stdarg.h>     // ���� ���� �Լ�(vsnprintf ��)�� ó���ϱ� ���� ��� ����
#include <ctype.h>      // ���� ó�� �Լ�(tolower)�� ����ϱ� ���� ��� ����

#ifdef _WIN32 // �ü���� windows�� ���
#include <windows.h>    // windows api �Լ�(Sleep, SetConsoleCursorPosition ��)�� ����ϱ� ���� ��� ����
#include <conio.h>      // �ܼ� ����� �Լ�(_getch)�� ����ϱ� ���� ��� ����
#define sleep(ms) Sleep(ms) // Sleep �Լ��� �и��� ������ ����
#else // windows�� �ƴ� �ٸ� �ü��(linux, macos ��)�� ���
#include <unistd.h>     // posix �ü�� api(usleep, read)�� ����ϱ� ���� ��� ����
#include <termios.h>    // �͹̳� ����(���� ��Ȱ��ȭ ��)�� ���� ��� ����
#define sleep(ms) usleep(ms * 1000) // usleep �Լ��� �и��� ������ ���� (usleep�� ����ũ���� ����)
#endif

// =============================================================================
// --- 1. ��� �� ���� ���� ���� ---
// =============================================================================
// --- �ý��� ��� ---
#define TRUE 1                  // �Ҹ��� ��(true) �� ����
#define FALSE 0                 // �Ҹ��� ����(false) �� ����
#define INPUT_BUFFER_SIZE 100   // ����� �Է� ������ ũ�� ����
#define DISPLAY_BUFFER_SIZE 16384 // ȭ�� �������� ���� ���� ũ�� ���� (���� ����ȭ��)

// --- ANSI �÷� �ڵ� ---
#define C_NRM "\x1b[0m"       // �⺻ �������� ����
#define C_RED "\x1b[31m"       // ������
#define C_GRN "\x1b[32m"       // �ʷϻ�
#define C_YEL "\x1b[33m"       // �����
#define C_BLU "\x1b[34m"       // �Ķ���
#define C_MAG "\x1b[35m"       // ����Ÿ��
#define C_CYN "\x1b[36m"       // �þȻ�
#define C_WHT "\x1b[37m"       // ���
#define C_GRY "\x1b[90m"       // ȸ��
#define C_B_RED "\x1b[1;31m"   // ���� ������ (����)
#define C_B_GRN "\x1b[1;32m"   // ���� �ʷϻ� (����)
#define C_B_YEL "\x1b[1;33m"   // ���� ����� (����)
#define C_B_MAG "\x1b[1;35m"   // ���� ����Ÿ�� (����)
#define C_B_CYN "\x1b[1;36m"   // ���� �þȻ� (����)
#define C_B_WHT "\x1b[1;37m"   // ���� ��� (����)

// --- �ùķ��̼� �׸��� �� ������Ʈ ��� ---
#define GRID_WIDTH 37           // �׸��� ���� �ʺ�
#define GRID_HEIGHT 12          // �׸��� ���� ����
#define MAX_AGENTS 3            // �ùķ��̼ǿ� �����ϴ� �ִ� ������Ʈ(�κ�) ��
#define MAX_GOALS (GRID_WIDTH * GRID_HEIGHT) // ������ �ִ� ��ǥ ����(���� ����) ��
#define INF DBL_MAX             // ���Ѵ� ���� double�� �ִ밪���� ���� (��� ��꿡 ���)
#define NUM_DIRECTIONS 4        // ������Ʈ�� �̵��� �� �ִ� ������ �� (��, ��, ��, ��)

// --- �ùķ��̼� ���� ��� ---
#define DISTANCE_BEFORE_CHARGE 300.0 // ������ �ʿ������� ���� �̵� �Ÿ�
#define CHARGE_TIME 20               // ������ �ɸ��� �ð� (�ùķ��̼� ���� ����)
#define PARKING_TIME 5               // ���� �۾��� �ҿ�Ǵ� �ð� (�ùķ��̼� ���� ����)
#define COLLECTION_TIME 5            // ���� ���� �۾��� �ҿ�Ǵ� �ð� (�ùķ��̼� ���� ����)
#define ROTATION_TIME 2              // [ADDED] ȸ���� �ҿ�Ǵ� �ð� (�ùķ��̼� ���� ����)
#define MAX_CHARGE_STATIONS 10       // �ִ� ������ ����
#define MAX_PHASES 20                // ����� ���� �ó��������� ���� ������ �ִ� �ܰ� ��
#define REALTIME_MODE_TIMELIMIT 10000 // �ǽð� ����� �ִ� �ùķ��̼� �ð�
#define MAX_TASKS 50                 // �۾� ť�� ������ �� �ִ� �ִ� �۾� ��
#define MAX_SPEED_MULTIPLIER 100.0f  // �ùķ��̼� �ִ� ���
#define EVENT_GENERATION_INTERVAL 5 // �ǽð� ��忡�� �̺�Ʈ(����/���� ��û)�� �߻��ϴ� �ð� ����

// --- �α� �� UI ��� ---
#define LOG_BUFFER_LINES 5      // ȭ�鿡 ǥ���� �α� �޽����� �ִ� �� ��
#define LOG_BUFFER_WIDTH 256    // �� �α� �޽����� �ִ� ����
#define STATUS_STRING_WIDTH 25  // ������Ʈ ���� ���ڿ��� ����� ���� ���� �ʺ� (UI ���Ŀ�)

// =============================================================================
// --- 2. ����ü ���� ---
// =============================================================================
// --- �⺻ ����ü ---
/** @struct Key
 * @brief �켱���� ť(priority queue)���� ����� �켱������ �����ϴ� Ű ��.
 */
typedef struct { double k1; double k2; } Key;

/** @struct Node
 * @brief �׸��� ���� �� ��(���)�� ��Ÿ���� ����ü.
 */
typedef struct Node {
    int x, y;                   // ����� �׸��� �� x, y ��ǥ
    double g, rhs;              // D* Lite �˰��򿡼� ����ϴ� g��(���� ���)�� rhs��(���� ���)
    Key key;                    // �켱���� ť������ ������ ���� Ű ��
    int is_obstacle;            // �� ��尡 ��ֹ����� ���� (TRUE/FALSE)
    int is_goal;                // �� ��尡 ���� ����(��ǥ)���� ����
    int is_temp;                // �ӽ� ��ֹ��� �����Ǿ����� ���� (�ٸ� ������Ʈ ȸ�ǿ�)
    int is_parked;              // �� ��忡 ������ �����Ǿ� �ִ��� ����
    int reserved_by_agent;      // � ������Ʈ�� ���� ����Ǿ����� (������Ʈ ID, -1�� ���� ����)
    int in_pq;                  // ���� �켱���� ť�� ���ԵǾ� �ִ��� ����
    int pq_index;               // �켱���� ť(��) �������� �ε���
} Node;

/** @enum AgentState
 * @brief ������Ʈ�� ���� ���¸� ��Ÿ���� ������.
 */
typedef enum {
    IDLE,                       // ��� ����
    ROTATING,                   // [ADDED] ȸ�� ��
    GOING_TO_PARK,              // �����Ϸ� ���� ��
    PERFORMING_PARKING,         // ���� �۾� ���� ��
    RETURNING_HOME_EMPTY,       // ���� �� �� ������ ������ �����ϴ� ��
    GOING_TO_COLLECT,           // ������ ���� ������ ���� ��
    PERFORMING_COLLECTION,      // ���� ���� �۾� ���� ��
    RETURNING_WITH_CAR,         // ���� �ư� ���� �������� ���� ��
    GOING_TO_CHARGE,            // �����ҷ� ���� ��
    CHARGING,                   // ���� ��
    RETURNING_HOME_MAINTENANCE  // ���� �� ������ �����ϴ� ����
} AgentState;

/** @enum AgentDirection
 * @brief [ADDED] ������Ʈ�� �ٶ󺸴� ������ ��Ÿ���� ������.
 */
typedef enum {
    NORTH,      // ���� (y-1)
    EAST,       // ���� (x+1)
    SOUTH,      // ���� (y+1)
    WEST,       // ���� (x-1)
    STATIONARY  // ���� ����
} AgentDirection;

/** @struct Agent
 * @brief �ϳ��� ������Ʈ(�κ�)�� ��Ÿ���� ����ü.
 */
typedef struct Agent {
    int id;                     // ������Ʈ�� ���� ID (0, 1, 2)
    char symbol;                // �ʿ� ǥ�õ� ������Ʈ�� ��ȣ ('A', 'B', 'C')
    Node* pos;                  // ���� ��ġ�� ����Ű�� ��� ������
    Node* home_base;            // ������Ʈ�� ����(������)�� ����Ű�� ���
    Node* goal;                 // ���� ��ǥ ������ ����Ű�� ��� ������
    AgentState state;           // ������Ʈ�� ���� ����
    AgentState previous_state;  // [ADDED] ȸ�� �� �ӽ� ���� ������ ������ �ִ� ����
    AgentDirection facing_direction; // [ADDED] ���� �ٶ󺸴� ����
    Node* intended_next_pos;    // [ADDED] ȸ�� �� �̵��� ���� ��ġ
    double total_distance_traveled; // �� �̵� �Ÿ� (���� �ʿ� ���� �Ǵܿ� ���)
    int charge_timer;           // ���� �ܿ� �ð�
    int action_timer;           // ����/����/ȸ�� �۾� �ܿ� �ð�
} Agent;

/** @struct PriorityQueue
 * @brief �ּ� ��(min-heap)���� ������ �켱���� ť.
 */
typedef struct {
    Node** nodes;   // ��� �����͸� �����ϴ� �迭
    int size;       // ���� ť�� ����� ����� ����
    int capacity;   // ť�� �ִ� �뷮
} PriorityQueue;

/** @enum PhaseType
 * @brief ����� ���� �ó������� �� �ܰ� ������ ��Ÿ���� ������.
 */
typedef enum { PARK_PHASE, EXIT_PHASE } PhaseType;

/** @struct DynamicPhase
 * @brief ����� ���� �ó������� �� �ܰ踦 �����ϴ� ����ü.
 */
typedef struct {
    PhaseType type;             // �ܰ��� ���� (���� �Ǵ� ����)
    int task_count;             // �� �ܰ迡�� �����ؾ� �� �۾�(����)�� ��
    char type_name[10];         // �ܰ� ������ ���ڿ��� ���� (UI ǥ�ÿ�)
} DynamicPhase;

/** @enum TaskType
 * @brief �ǽð� ��忡�� �����Ǵ� �۾��� ������ ��Ÿ���� ������.
 */
typedef enum { TASK_NONE, TASK_PARK, TASK_EXIT } TaskType;

/** @struct TaskNode
 * @brief �۾� ť�� ���� ��带 ��Ÿ���� ���� ����Ʈ ����ü.
 */
typedef struct TaskNode {
    TaskType type;
    struct TaskNode* next;
} TaskNode;

/** @enum SimulationMode
 * @brief �ùķ��̼��� ��ü ���� ��带 ��Ÿ���� ������.
 */
typedef enum { MODE_UNINITIALIZED, MODE_CUSTOM, MODE_REALTIME } SimulationMode;

// --- ���ȭ�� ����ü ---
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

typedef struct {
    PriorityQueue pq;
    Node* start_node;
    Node* goal_node;
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
// --- 3. �Լ� ������Ÿ�� ���� ---
// =============================================================================
// --- ��� ����/�Ҹ� �Լ� ---
Simulation* simulation_create();
void simulation_destroy(Simulation* sim);

// --- �ý���, UI �� �α� �Լ� ---
void system_enable_virtual_terminal();
void ui_clear_screen_optimized();
int simulation_setup(Simulation* sim);
void logger_log(Logger* logger, const char* format, ...);

// --- �켱���� ť (Priority Queue) �Լ� ---
void pq_init(PriorityQueue* pq, int capacity);
void pq_free(PriorityQueue* pq);
void pq_push(PriorityQueue* pq, Node* node);
Node* pq_pop(PriorityQueue* pq);
void pq_remove(PriorityQueue* pq, Node* node);
int pq_contains(const Node* node);
Key pq_top_key(const PriorityQueue* pq);

// --- �׸��� (GridMap) ���� �Լ� ---
int grid_is_valid_coord(int x, int y);
int grid_is_node_blocked(const GridMap* map, const AgentManager* agent_manager, const Node* n);

// --- ��� Ž�� (Pathfinder) �Լ� ---
Pathfinder* pathfinder_create(Node* start, Node* goal);
void pathfinder_destroy(Pathfinder* pf);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, AgentManager* agent_manager);
Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager, Node* current_node);

// --- ������Ʈ (AgentManager) ���� �Լ� ---
void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]);
void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger);
void agent_manager_update_charge_state(AgentManager* manager, Logger* logger);
void agent_manager_update_action_state(AgentManager* manager, Logger* logger, GridMap* map); // [MODIFIED] map ����

// --- �ùķ��̼� (Simulation) ���� �Լ� ---
void simulation_run(Simulation* sim);

// =============================================================================
// --- 4. �ý���, UI �� �α� ���� ---
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
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr icanon");
    if (read(0, &buf, 1) < 0) perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~icanon");
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
        printf(C_B_RED "\n�߸��� �Է��Դϴ�. ��ȿ�� Ű�� �����ּ���. (%s)\n" C_NRM, valid_chars);
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
        printf(C_B_RED "�߸��� �Է��Դϴ�. %d���� %d ������ ������ �Է��ϼ���.\n" C_NRM, min, max);
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
        printf(C_B_RED "�߸��� �Է��Դϴ�. %.1f���� %.1f ������ ���ڸ� �Է��ϼ���.\n" C_NRM, min, max);
    }
}

static int simulation_setup_custom_scenario(ScenarioManager* scenario) {
    printf(C_B_WHT "--- ����� ���� �ó����� ���� ---\n" C_NRM);
    scenario->num_phases = get_integer_input(C_YEL "�� �ܰ� ���� �Է��ϼ��� (1-20, 0=���): " C_NRM, 0, MAX_PHASES);
    if (scenario->num_phases == 0) return 0;

    for (int i = 0; i < scenario->num_phases; i++) {
        printf(C_B_CYN "\n--- %d/%d �ܰ� ���� ---\n" C_NRM, i + 1, scenario->num_phases);
        printf("a. %s����%s\n", C_YEL, C_NRM);
        printf("b. %s����%s\n", C_CYN, C_NRM);
        char type_choice = get_char_input("�ܰ� ������ �����ϼ���: ", "ab");
        scenario->phases[i].task_count = get_integer_input("�� �ܰ迡�� ó���� ���� ���� �Է��ϼ���: ", 1, 100);

        if (type_choice == 'a') {
            scenario->phases[i].type = PARK_PHASE;
            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "����");
        }
        else {
            scenario->phases[i].type = EXIT_PHASE;
            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "����");
        }
        printf(C_GRN "%d�ܰ� ���� �Ϸ�: %s %d��.\n" C_NRM, i + 1, scenario->phases[i].type_name, scenario->phases[i].task_count);
    }
    printf(C_B_GRN "\n--- �ó����� ������ �Ϸ�Ǿ����ϴ�! ---\n" C_NRM);
    sleep(1500);
    return 1;
}

static int simulation_setup_realtime(ScenarioManager* scenario) {
    printf(C_B_WHT "--- �ǽð� �ùķ��̼� ���� ---\n" C_NRM);
    while (TRUE) {
        scenario->park_chance = get_integer_input("\n���� ��û �߻� Ȯ��(0~100)�� �Է��ϼ���: ", 0, 100);
        scenario->exit_chance = get_integer_input("���� ��û �߻� Ȯ��(0~100)�� �Է��ϼ���: ", 0, 100);
        if ((scenario->park_chance + scenario->exit_chance) <= 100) break;
        printf(C_B_RED "������ ���� Ȯ���� ���� 100�� ���� �� �����ϴ�.\n" C_NRM);
    }
    printf(C_B_GRN "\n���� �Ϸ�: ���� Ȯ�� %d%%, ���� Ȯ�� %d%%\n" C_NRM, scenario->park_chance, scenario->exit_chance);
    sleep(1500);
    return 1;
}

static int simulation_setup_speed(ScenarioManager* scenario) {
    printf(C_B_WHT "\n--- �ùķ��̼� �ӵ� ���� ---\n" C_NRM);
    scenario->speed_multiplier = get_float_input("���ϴ� ����� �Է��ϼ��� (1.0 ~ 100.0): ", 1.0f, MAX_SPEED_MULTIPLIER);
    scenario->simulation_speed = (int)(100.0f / scenario->speed_multiplier);
    if (scenario->simulation_speed < 1) scenario->simulation_speed = 1;
    printf(C_B_GRN "\n--- �ùķ��̼��� %.1fx ������� �����մϴ�... ---\n" C_NRM, scenario->speed_multiplier);
    sleep(1500);
    return 1;
}

int simulation_setup(Simulation* sim) {
    ui_clear_screen_optimized();
    printf(C_B_WHT "--- �ùķ��̼� ��� ���� ---\n" C_NRM);
    printf("a. %s����� ���� �ó�����%s\n", C_YEL, C_NRM);
    printf("b. %s�ǽð� �ùķ��̼�%s\n", C_CYN, C_NRM);
    printf("q. %s����%s\n\n", C_RED, C_NRM);

    char choice = get_char_input("������ �ó����� ���ڸ� �Է��ϼ���: ", "abq");
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
        view[cs->y][cs->x] = 'e';
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

    written = snprintf(buf_ptr, remaining_size, C_B_WHT "\n--- D* Lite Parking Simulation (v1.2) ---\n" C_NRM);
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
    return buf_ptr - buffer;
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
        written = snprintf(buf_ptr, remaining_size, "--- Real-Time Simulation [Speed: %.1fx] ---\n", scenario->speed_multiplier);
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
            C_B_GRN, park_requests, C_NRM, C_B_YEL, exit_requests, C_NRM
        );
        buf_ptr += written; remaining_size -= written;
    }
    written = snprintf(buf_ptr, remaining_size, "Parked Cars: %d/%d\n" C_NRM, agent_manager->total_cars_parked, map->num_goals);
    buf_ptr += written; remaining_size -= written;

    written = grid_map_render_to_buffer(buf_ptr, remaining_size, map, agent_manager);
    buf_ptr += written; remaining_size -= written;

    // [MODIFIED] ����, ����, ���� ���ڿ� �߰�
    const char* agent_state_strings[] = { "���", "ȸ�� ��", "���� �������� �̵�", "���� �۾� ��", "���� ����(�� ��)", "�����Ϸ� �̵�", "���� �۾� ��", "���� ��(���� ž��)", "�����ҷ� �̵�", "���� ��", "���� ����(���� ��)" };
    const char* agent_state_colors[] = { C_GRY, C_B_CYN, C_YEL, C_B_YEL, C_CYN, C_YEL, C_B_YEL, C_GRN, C_B_RED, C_RED, C_CYN };
    const char* direction_symbols[] = { "^", ">", "v", "<", " " }; // ��, ��, ��, ��, ����

    for (int i = 0; i < MAX_AGENTS; i++) {
        const Agent* agent = &agent_manager->agents[i];
        const char* agent_color = (i == 0) ? C_B_CYN : (i == 1) ? C_B_YEL : C_B_MAG;
        char status_buffer[100];

        // [MODIFIED] ȸ�� ���� Ÿ�̸� ǥ�� ���� �߰�
        if (agent->state == CHARGING) {
            snprintf(status_buffer, sizeof(status_buffer), "���� ��... (%d)", agent->charge_timer);
        }
        else if (agent->state == PERFORMING_PARKING) {
            snprintf(status_buffer, sizeof(status_buffer), "���� �۾� ��... (%d)", agent->action_timer);
        }
        else if (agent->state == PERFORMING_COLLECTION) {
            snprintf(status_buffer, sizeof(status_buffer), "���� �۾� ��... (%d)", agent->action_timer);
        }
        else if (agent->state == ROTATING) {
            snprintf(status_buffer, sizeof(status_buffer), "ȸ�� ��... (%d)", agent->action_timer);
        }
        else {
            snprintf(status_buffer, sizeof(status_buffer), "%s", agent_state_strings[agent->state]);
        }

        // [MODIFIED] ������Ʈ ���� ǥ�� �߰�
        written = snprintf(buf_ptr, remaining_size, "%sAgent %c%s (%s%s%s): (%2d,%d) ", agent_color, agent->symbol, C_NRM, C_B_WHT, direction_symbols[agent->facing_direction], C_NRM, agent->pos->x, agent->pos->y);
        buf_ptr += written; remaining_size -= written;

        if (agent->goal) {
            written = snprintf(buf_ptr, remaining_size, "-> (%2d,%d) ", agent->goal->x, agent->goal->y);
        }
        else {
            written = snprintf(buf_ptr, remaining_size, "-> ����    ");
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

Logger* logger_create() {
    Logger* logger = (Logger*)calloc(1, sizeof(Logger));
    if (!logger) { perror("Logger �Ҵ� ����"); exit(1); }
    return logger;
}

void logger_destroy(Logger* logger) { if (logger) free(logger); }

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

// ... (Priority Queue �����δ� ���� ������ �����Ƿ� ����) ...
// =============================================================================
// --- 5. ��ƿ��Ƽ ���� (�켱���� ť) ---
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
// =============================================================================
// --- 6. �� ���� ���� (GridMap) ---
// =============================================================================
static void grid_map_load_from_embedded_map(GridMap* map, AgentManager* agent_manager) {
    const char* embedded_map_data =
        "1111111111111111111111111111111111111\n"
        "C01GGG1GG1GGG1GGG1GGG1GGG1GGG1G11G111\n"
        "A000000000000000000000000000000000001\n"
        "B0000000000000000000000000000000000001\n"
        "1111GG1GG1GGG10001GGG1GGG1GGG1100e111\n"
        "111111111111110001GGG1GGG1GGG11001111\n"
        "100000000000000000000000000000000e111\n"
        "100000000000000000000000000000000e111\n"
        "11100001111111GGG1GGG1GGG1GGG1GG11111\n"
        "1000000000000000000000000000000000001\n"
        "1GG1GG1GG1GG1GG1GG1GG1GG1GG1GG1GG1GG1\n"
        "1111111111111111111111111111111111111\n";

    map->num_goals = 0;
    map->num_charge_stations = 0;
    int map_idx = 0;
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            char ch;
            do { ch = embedded_map_data[map_idx++]; } while (ch == '\n' || ch == '\r');
            if (ch == '\0') {
                fprintf(stderr, C_B_RED "����: ����� �� �����Ͱ� ���󺸴� ª���ϴ�.\n" C_NRM);
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
            case 'e': if (map->num_charge_stations < MAX_CHARGE_STATIONS) map->charge_stations[map->num_charge_stations++] = n; break;
            }
        }
    }
    if (map->num_charge_stations == 0) {
        fprintf(stderr, C_B_RED "����: �� �����Ϳ� ������('e')�� �����ϴ�.\n" C_NRM);
        exit(1);
    }
}

GridMap* grid_map_create(AgentManager* agent_manager) {
    GridMap* map = (GridMap*)calloc(1, sizeof(GridMap));
    if (!map) { perror("GridMap �Ҵ� ����"); exit(1); }
    grid_map_load_from_embedded_map(map, agent_manager);
    return map;
}

void grid_map_destroy(GridMap* map) { if (map) free(map); }

int grid_is_valid_coord(int x, int y) {
    return x >= 0 && x < GRID_WIDTH&& y >= 0 && y < GRID_HEIGHT;
}

int grid_is_node_blocked(const GridMap* map, const AgentManager* agent_manager, const Node* n) {
    if (n->is_obstacle || n->is_parked || n->is_temp) {
        return TRUE;
    }
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (agent_manager->agents[i].pos == n &&
            (agent_manager->agents[i].state == CHARGING ||
                agent_manager->agents[i].state == PERFORMING_PARKING ||
                agent_manager->agents[i].state == PERFORMING_COLLECTION ||
                agent_manager->agents[i].state == ROTATING)) { // [MODIFIED] ȸ�� �߿��� ���� ������ ����
            return TRUE;
        }
    }
    return FALSE;
}

// ... (ScenarioManager, Pathfinder �����δ� ���� ���� ����) ...
// =============================================================================
// --- 7. �ó����� �� �۾� ���� ���� (ScenarioManager) ---
// =============================================================================
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
    if (!manager) { perror("ScenarioManager �Ҵ� ����"); exit(1); }
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
static void add_task_to_queue(ScenarioManager* scenario, TaskType type) {
    if (scenario->task_count >= MAX_TASKS) return;
    TaskNode* new_task = (TaskNode*)malloc(sizeof(TaskNode));
    if (!new_task) { perror("Failed to allocate memory for new task"); return; }
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
// =============================================================================
// --- 8. ��� Ž�� ���� (Pathfinder - D* Lite) ---
// =============================================================================
static double heuristic(const Node* a, const Node* b) { return fabs(a->x - b->x) + fabs(a->y - b->y); }
static double euclidean_distance(const Node* a, const Node* b) { return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2)); }
static Key calculate_key(const Pathfinder* pf, const Node* n) {
    double m = fmin(n->g, n->rhs);
    return (Key) { m + heuristic(pf->start_node, n), m };
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
    if (pq_contains(u)) { pq_remove(&pf->pq, u); }
    if (fabs(u->g - u->rhs) > 1e-9) {
        u->key = calculate_key(pf, u);
        pq_push(&pf->pq, u);
    }
}
Pathfinder* pathfinder_create(Node* start, Node* goal) {
    Pathfinder* pf = (Pathfinder*)malloc(sizeof(Pathfinder));
    if (!pf) return NULL;
    pf->start_node = start;
    pf->goal_node = goal;
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
    while (pf->pq.size > 0 &&
        (compare_keys(pq_top_key(&pf->pq), calculate_key(pf, pf->start_node)) < 0 ||
            fabs(pf->start_node->rhs - pf->start_node->g) > 1e-9)) {
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
Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager, Node* current_node) {
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
            double cost = 1.0 + neighbor->g;
            if (cost < min_g) {
                min_g = cost;
                best_next_node = neighbor;
                min_dist_to_goal = euclidean_distance(neighbor, pf->goal_node);
            }
            else if (fabs(cost - min_g) < 1e-9) {
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
// --- 9. ������Ʈ ���� �� ���� ���� (AgentManager) ---
// =============================================================================
AgentManager* agent_manager_create() {
    AgentManager* manager = (AgentManager*)calloc(1, sizeof(AgentManager));
    if (!manager) { perror("AgentManager �Ҵ� ����"); exit(1); }
    for (int i = 0; i < MAX_AGENTS; i++) {
        manager->agents[i].id = i;
        manager->agents[i].symbol = 'A' + i;
        manager->agents[i].state = IDLE;
        manager->agents[i].action_timer = 0;
        manager->agents[i].facing_direction = STATIONARY; // [ADDED] ���� �ʱ�ȭ
        manager->agents[i].intended_next_pos = NULL;      // [ADDED] �ǵ� ��ġ �ʱ�ȭ
    }
    return manager;
}

void agent_manager_destroy(AgentManager* manager) { if (manager) free(manager); }

static double calculate_path_cost(Agent* agent, Node* goal, GridMap* map, AgentManager* agent_manager) {
    Pathfinder* pf = pathfinder_create(agent->pos, goal);
    if (!pf) return INF;
    pathfinder_compute_shortest_path(pf, map, agent_manager);
    double cost = agent->pos->g;
    pathfinder_destroy(pf);
    return cost;
}

static Node* select_best_parking_spot(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
    double best_cost = INF;
    Node* best_goal = NULL;
    for (int j = 0; j < map->num_goals; j++) {
        Node* g = map->goals[j];
        if (g->is_parked || (g->reserved_by_agent != -1 && g->reserved_by_agent != agent->id)) continue;
        double cost = calculate_path_cost(agent, g, map, agent_manager);
        if (cost < best_cost) {
            best_cost = cost;
            best_goal = g;
        }
    }
    if (best_goal) {
        logger_log(logger, "[%sPlan%s] Agent %c, ���� ���� (%d,%d) ���� (���: %.1f)", C_CYN, C_NRM, agent->symbol, best_goal->x, best_goal->y, best_cost);
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
        double cost = calculate_path_cost(agent, g, map, agent_manager);
        g->is_parked = TRUE;

        if (cost < best_cost) {
            best_cost = cost;
            best_spot = g;
        }
    }
    if (best_spot) {
        logger_log(logger, "[%sPlan%s] Agent %c, ���� ���� (%d,%d) ���� (���: %.1f)", C_CYN, C_NRM, agent->symbol, best_spot->x, best_spot->y, best_cost);
    }
    return best_spot;
}

static Node* select_best_charge_station(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
    double best_cost = INF;
    Node* best_station = NULL;
    for (int i = 0; i < map->num_charge_stations; i++) {
        Node* station = map->charge_stations[i];
        if (station->reserved_by_agent != -1 && station->reserved_by_agent != agent->id) continue;
        double cost = calculate_path_cost(agent, station, map, agent_manager);
        if (cost < best_cost) {
            best_cost = cost;
            best_station = station;
        }
    }
    if (best_station) {
        logger_log(logger, "[%sPlan%s] Agent %c, ������ (%d,%d) ���� (���: %.1f)", C_CYN, C_NRM, agent->symbol, best_station->x, best_station->y, best_cost);
    }
    return best_station;
}

static void agent_set_goal(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
    if (agent->state == RETURNING_HOME_EMPTY && agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
        if (agent->goal) {
            agent->goal->reserved_by_agent = -1;
            agent->goal = NULL;
        }
        logger_log(logger, "[%sCharge%s] Agent %c ���� �ʿ�! ��ǥ�� �����ҷ� ��� ����.", C_B_YEL, C_NRM, agent->symbol);
        agent->state = GOING_TO_CHARGE;
    }

    // [MODIFIED] ȸ�� ���̰ų� �ٸ� �۾� ���� �� ��ǥ �缳�� ����
    if (agent->state == IDLE || agent->state == CHARGING || agent->state == PERFORMING_PARKING || agent->state == PERFORMING_COLLECTION || agent->state == ROTATING || agent->goal != NULL) return;

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
        agent->facing_direction = STATIONARY;
        logger_log(logger, "[%sInfo%s] Agent %c: ���� ��ǥ ����. ��� ���·� ��ȯ.", C_YEL, C_NRM, agent->symbol);
    }
}

void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        AgentState s = agent->state;
        if (agent->goal == NULL && s != IDLE && s != CHARGING && s != PERFORMING_PARKING && s != PERFORMING_COLLECTION && s != ROTATING) {
            agent_set_goal(agent, map, manager, logger);
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        next_pos[i] = manager->agents[i].pos;
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        AgentState s = agent->state;
        if (s == IDLE || s == CHARGING || s == PERFORMING_PARKING || s == PERFORMING_COLLECTION || s == ROTATING || agent->goal == NULL) {
            continue;
        }

        Node* obstacles_to_clear[MAX_AGENTS];
        int obs_count = 0;
        for (int j = 0; j < MAX_AGENTS; j++) {
            if (i == j) continue;
            Node* obs_node = (j < i) ? next_pos[j] : manager->agents[j].pos;
            if (obs_node) {
                obs_node->is_temp = TRUE;
                obstacles_to_clear[obs_count++] = obs_node;
            }
        }

        int goal_was_parked = (agent->state == GOING_TO_COLLECT && agent->goal->is_parked);
        if (goal_was_parked) agent->goal->is_parked = FALSE;

        Pathfinder* pf = pathfinder_create(agent->pos, agent->goal);
        pathfinder_compute_shortest_path(pf, map, manager);
        next_pos[i] = pathfinder_get_next_step(pf, map, manager, agent->pos);
        pathfinder_destroy(pf);

        if (goal_was_parked) agent->goal->is_parked = TRUE;

        for (int k = 0; k < obs_count; k++) {
            if (obstacles_to_clear[k]) obstacles_to_clear[k]->is_temp = FALSE;
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        for (int j = i + 1; j < MAX_AGENTS; j++) {
            if (next_pos[i] == manager->agents[i].pos && next_pos[j] == manager->agents[j].pos) continue;

            if (next_pos[i] == next_pos[j]) {
                logger_log(logger, "[%sAvoid%s] �浹 ����! Agent %c ���.", C_B_RED, C_NRM, manager->agents[j].symbol);
                next_pos[j] = manager->agents[j].pos;
            }
            else if (next_pos[i] == manager->agents[j].pos && next_pos[j] == manager->agents[i].pos) {
                logger_log(logger, "[%sAvoid%s] ���� �浹 ����! Agent %c ���.", C_B_RED, C_NRM, manager->agents[j].symbol);
                next_pos[j] = manager->agents[j].pos;
            }
        }
    }
}

void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        AgentState s = agent->state;

        if (s == IDLE || s == CHARGING || s == PERFORMING_PARKING || s == PERFORMING_COLLECTION || s == ROTATING || !agent->goal || agent->pos != agent->goal) {
            continue;
        }

        Node* reached_goal = agent->goal;
        if (s != GOING_TO_CHARGE && s != RETURNING_HOME_EMPTY && s != RETURNING_WITH_CAR && s != RETURNING_HOME_MAINTENANCE) {
            reached_goal->reserved_by_agent = -1;
        }
        agent->goal = NULL;

        switch (s) {
        case GOING_TO_PARK:
            reached_goal->is_parked = TRUE;
            manager->total_cars_parked++;
            logger_log(logger, "[%sPark%s] Agent %c, ���� �۾� ���� at (%d,%d)... (%d steps)", C_B_YEL, C_NRM, agent->symbol, reached_goal->x, reached_goal->y, PARKING_TIME);
            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == PARK_PHASE) {
                scenario->tasks_completed_in_phase++;
            }
            agent->state = PERFORMING_PARKING;
            agent->action_timer = PARKING_TIME;
            break;

        case RETURNING_HOME_EMPTY:
            logger_log(logger, "[%sInfo%s] Agent %c, ���� �۾� �� ���� ���� �Ϸ�.", C_CYN, C_NRM, agent->symbol);
            agent->state = IDLE;
            agent->facing_direction = STATIONARY;
            reached_goal->reserved_by_agent = -1;
            break;

        case GOING_TO_COLLECT:
            reached_goal->is_parked = FALSE;
            manager->total_cars_parked--;
            logger_log(logger, "[%sExit%s] Agent %c, ���� ���� �۾� ���� at (%d,%d)... (%d steps)", C_B_YEL, C_NRM, agent->symbol, reached_goal->x, reached_goal->y, COLLECTION_TIME);
            agent->state = PERFORMING_COLLECTION;
            agent->action_timer = COLLECTION_TIME;
            break;

        case RETURNING_WITH_CAR:
            logger_log(logger, "[%sExit%s] Agent %c, ���� ���� �Ϸ�.", C_GRN, C_NRM, agent->symbol);
            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == EXIT_PHASE) {
                scenario->tasks_completed_in_phase++;
            }
            agent->state = IDLE;
            agent->facing_direction = STATIONARY;
            reached_goal->reserved_by_agent = -1;
            break;

        case GOING_TO_CHARGE:
            logger_log(logger, "[%sCharge%s] Agent %c, ���� ����. (%d steps)", C_B_YEL, C_NRM, agent->symbol, CHARGE_TIME);
            agent->state = CHARGING;
            agent->charge_timer = CHARGE_TIME;
            break;

        case RETURNING_HOME_MAINTENANCE:
            logger_log(logger, "[%sInfo%s] Agent %c, ���� �� ���� ���� �Ϸ�.", C_CYN, C_NRM, agent->symbol);
            agent->state = IDLE;
            agent->facing_direction = STATIONARY;
            reached_goal->reserved_by_agent = -1;
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
                logger_log(logger, "[%sCharge%s] Agent %c ���� �Ϸ�.", C_B_GRN, C_NRM, agent->symbol);
                agent->total_distance_traveled = 0.0;
                agent->state = RETURNING_HOME_MAINTENANCE;
                if (agent->pos) agent->pos->reserved_by_agent = -1;
                agent->goal = NULL;
            }
        }
    }
}

// [ADDED] ���� ��ġ�� ���� ��ġ�� ������� �ʿ��� ������ ����ϴ� ���� �Լ�
static AgentDirection calculate_required_direction(Node* current, Node* next) {
    if (next->y < current->y) return NORTH;
    if (next->y > current->y) return SOUTH;
    if (next->x < current->x) return WEST;
    if (next->x > current->x) return EAST;
    return STATIONARY;
}


/**
 * @brief [MODIFIED] ����, ����, ȸ�� �۾� ���� ������Ʈ�� ���¸� ������Ʈ�մϴ�.
 */
void agent_manager_update_action_state(AgentManager* manager, Logger* logger, GridMap* map) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (agent->state == PERFORMING_PARKING) {
            agent->action_timer--;
            if (agent->action_timer <= 0) {
                logger_log(logger, "[%sPark%s] Agent %c, ���� �۾� �Ϸ�. ������ �����մϴ�.", C_GRN, C_NRM, agent->symbol);
                agent->state = RETURNING_HOME_EMPTY;
                agent->goal = NULL;
            }
        }
        else if (agent->state == PERFORMING_COLLECTION) {
            agent->action_timer--;
            if (agent->action_timer <= 0) {
                logger_log(logger, "[%sExit%s] Agent %c, ���� �۾� �Ϸ�. ���� �������� �̵��մϴ�.", C_GRN, C_NRM, agent->symbol);
                agent->state = RETURNING_WITH_CAR;
                agent->goal = NULL;
            }
        }
        else if (agent->state == ROTATING) { // [ADDED] ȸ�� ���� ó��
            agent->action_timer--;
            if (agent->action_timer <= 0) {
                AgentDirection required_dir = calculate_required_direction(agent->pos, agent->intended_next_pos);
                logger_log(logger, "[%sMove%s] Agent %c, ȸ�� �Ϸ�. �̵� �簳.", C_CYN, C_NRM, agent->symbol);

                // �浹 ��Ȯ��: ���� ȸ���ϴ� ���� ������ �� �������� ������ �� ����
                if (grid_is_node_blocked(map, manager, agent->intended_next_pos)) {
                    logger_log(logger, "[%sAvoid%s] Agent %c, ȸ�� �� �̵��Ϸ� ������ ��� ����. ���.", C_B_RED, C_NRM, agent->symbol);
                    agent->state = agent->previous_state; // ���� ���·� ����
                    agent->facing_direction = required_dir; // ������ ����
                    agent->intended_next_pos = NULL;
                }
                else {
                    agent->state = agent->previous_state; // ���� ���·� ����
                    agent->facing_direction = required_dir;
                    agent->pos = agent->intended_next_pos; // �����ص� ��ġ�� �̵�
                    agent->total_distance_traveled += 1.0;
                    agent->intended_next_pos = NULL;
                }
            }
        }
    }
}


// =============================================================================
// --- 10. �ùķ��̼� �ھ� ���� ���� ---
// =============================================================================
Simulation* simulation_create() {
    Simulation* sim = (Simulation*)calloc(1, sizeof(Simulation));
    if (!sim) { perror("Simulation �Ҵ� ����"); exit(1); }
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

static void simulation_update_state(Simulation* sim) {
    // ... (�� �Լ��� ���� ���� ����, ������ ���� ����) ...
    ScenarioManager* scenario = sim->scenario_manager;
    AgentManager* agent_manager = sim->agent_manager;
    GridMap* map = sim->map;
    Logger* logger = sim->logger;

    if (scenario->mode == MODE_CUSTOM) {
        if (scenario->current_phase_index >= scenario->num_phases) return;
        DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
        if (scenario->tasks_completed_in_phase >= phase->task_count) {
            logger_log(logger, "[%sPhase%s] %d�ܰ� (%s %d��) �Ϸ�!", C_B_YEL, C_NRM, scenario->current_phase_index + 1, phase->type_name, phase->task_count);
            scenario->current_phase_index++;
            scenario->tasks_completed_in_phase = 0;
            if (scenario->current_phase_index < scenario->num_phases) {
                DynamicPhase* next_phase = &scenario->phases[scenario->current_phase_index];
                logger_log(logger, "[%sPhase%s] %d�ܰ� ����: %s %d��.", C_B_YEL, C_NRM, scenario->current_phase_index + 1, next_phase->type_name, next_phase->task_count);
                sleep(1500);
            }
            return;
        }
    }
    else if (scenario->mode == MODE_REALTIME) {
        if (scenario->time_step > 0 && scenario->time_step % EVENT_GENERATION_INTERVAL == 0) {
            int event_chance = rand() % 100;
            if (event_chance < scenario->park_chance && agent_manager->total_cars_parked < map->num_goals) {
                logger_log(logger, "[%sEvent%s] ���ο� ���� ��û �߻�.", C_B_GRN, C_NRM);
                add_task_to_queue(scenario, TASK_PARK);
            }
            else if (event_chance < (scenario->park_chance + scenario->exit_chance) && agent_manager->total_cars_parked > 0) {
                logger_log(logger, "[%sEvent%s] ���ο� ���� ��û �߻�.", C_B_YEL, C_NRM);
                add_task_to_queue(scenario, TASK_EXIT);
            }
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &agent_manager->agents[i];
        if (agent->state == IDLE) {
            if (agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
                if (select_best_charge_station(agent, map, agent_manager, logger)) {
                    agent->state = GOING_TO_CHARGE;
                }
                else {
                    logger_log(logger, "[%sWarn%s] Agent %c ���� �ʿ��ϳ� ��� �����Ұ� ��� ��.", C_YEL, C_NRM, agent->symbol);
                }
                continue;
            }

            if (scenario->mode == MODE_CUSTOM) {
                if (scenario->current_phase_index >= scenario->num_phases) continue;
                DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];

                if (phase->type == PARK_PHASE) {
                    int active_tasks = 0;
                    for (int j = 0; j < MAX_AGENTS; j++) {
                        AgentState s = agent_manager->agents[j].state;
                        if (s == GOING_TO_PARK || s == PERFORMING_PARKING || s == RETURNING_HOME_EMPTY) {
                            active_tasks++;
                        }
                    }
                    if ((scenario->tasks_completed_in_phase + active_tasks) < phase->task_count && agent_manager->total_cars_parked < map->num_goals) {
                        agent->state = GOING_TO_PARK;
                        logger_log(logger, "[%sTask%s] Agent %c, �ű� ���� �۾� �Ҵ�.", C_CYN, C_NRM, agent->symbol);
                    }
                }
                else if (phase->type == EXIT_PHASE) {
                    int active_tasks = 0;
                    for (int j = 0; j < MAX_AGENTS; j++) {
                        AgentState s = agent_manager->agents[j].state;
                        if (s == GOING_TO_COLLECT || s == PERFORMING_COLLECTION || s == RETURNING_WITH_CAR) {
                            active_tasks++;
                        }
                    }
                    if ((scenario->tasks_completed_in_phase + active_tasks) < phase->task_count && agent_manager->total_cars_parked > 0) {
                        agent->state = GOING_TO_COLLECT;
                        logger_log(logger, "[%sTask%s] Agent %c, �ű� ���� �۾� �Ҵ�.", C_CYN, C_NRM, agent->symbol);
                    }
                }
            }
            else if (scenario->mode == MODE_REALTIME && scenario->task_count > 0) {
                int is_parking_lot_full = (agent_manager->total_cars_parked >= map->num_goals);
                TaskNode* current = scenario->task_queue_head;
                TaskNode* prev = NULL;

                while (current != NULL) {
                    int can_process = FALSE;
                    if (is_parking_lot_full) {
                        if (current->type == TASK_EXIT && agent_manager->total_cars_parked > 0) can_process = TRUE;
                    }
                    else {
                        if (current->type == TASK_PARK) can_process = TRUE;
                        else if (current->type == TASK_EXIT && agent_manager->total_cars_parked > 0) can_process = TRUE;
                    }

                    if (can_process) {
                        if (current->type == TASK_PARK) {
                            agent->state = GOING_TO_PARK;
                            logger_log(logger, "[%sTask%s] Agent %c, ���� �۾� �Ҵ�.", C_CYN, C_NRM, agent->symbol);
                        }
                        else {
                            agent->state = GOING_TO_COLLECT;
                            logger_log(logger, "[%sTask%s] Agent %c, ���� �۾� �Ҵ�.", C_CYN, C_NRM, agent->symbol);
                        }

                        if (prev == NULL) scenario->task_queue_head = current->next;
                        else prev->next = current->next;
                        if (current == scenario->task_queue_tail) scenario->task_queue_tail = prev;

                        free(current);
                        scenario->task_count--;
                        break;
                    }
                    prev = current;
                    current = current->next;
                }
            }
        }
    }
}

static int simulation_is_complete(const Simulation* sim) {
    const ScenarioManager* scenario = sim->scenario_manager;
    const AgentManager* agent_manager = sim->agent_manager;

    if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index >= scenario->num_phases) {
        for (int i = 0; i < MAX_AGENTS; i++) {
            if (agent_manager->agents[i].state != IDLE) return FALSE;
        }
        printf(C_B_GRN "\n��� �ó����� �ܰ谡 �Ϸ�Ǿ����ϴ�! �ùķ��̼��� �����մϴ�.\n" C_NRM);
        return TRUE;
    }

    if (scenario->mode == MODE_REALTIME && scenario->time_step >= REALTIME_MODE_TIMELIMIT) {
        printf(C_B_GRN "\n�ǽð� �ùķ��̼� �ð� ���ѿ� �����߽��ϴ�! �ùķ��̼��� �����մϴ�.\n" C_NRM);
        return TRUE;
    }
    return FALSE;
}

void simulation_run(Simulation* sim) {
    while (TRUE) { // ���� ����
        // 1. Ÿ�̸� ��� ���� ������Ʈ
        agent_manager_update_charge_state(sim->agent_manager, sim->logger);
        agent_manager_update_action_state(sim->agent_manager, sim->logger, sim->map);

        // 2. �۾� �Ҵ�
        simulation_update_state(sim);

        // 3. ��� ��ȹ �� �浹 ȸ��
        Node* next_pos[MAX_AGENTS];
        agent_manager_plan_and_resolve_collisions(sim->agent_manager, sim->map, sim->logger, next_pos);

        // 4. [MODIFIED] ȸ�� ���� �� �̵� ���� ����
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* agent = &sim->agent_manager->agents[i];
            AgentState s = agent->state;

            // Ÿ�̸Ӱ� ���� ���� ����(����, �۾�, ȸ��)������ ���ο� �̵��� �������� ����
            if (s == CHARGING || s == PERFORMING_PARKING || s == PERFORMING_COLLECTION || s == ROTATING) {
                continue;
            }

            if (agent->pos != next_pos[i] && next_pos[i] != NULL) { // �̵��� �ʿ��� ���
                AgentDirection required_dir = calculate_required_direction(agent->pos, next_pos[i]);

                // ȸ���� �ʿ����� Ȯ�� (���� ���¿��� ó�� �����̴� ���� ����)
                if (agent->facing_direction != required_dir && agent->facing_direction != STATIONARY) {
                    // [FIXED] logger�� sim->logger�� �����Ͽ� ���� �ذ�
                    logger_log(sim->logger, "[%sMove%s] Agent %c, ȸ�� �ʿ�. (%d steps)", C_YEL, C_NRM, agent->symbol, ROTATION_TIME);
                    agent->previous_state = agent->state; // ���� ���� ����
                    agent->state = ROTATING;              // ȸ�� ���·� ����
                    agent->action_timer = ROTATION_TIME;  // ȸ�� Ÿ�̸� ����
                    agent->intended_next_pos = next_pos[i]; // ȸ�� �� �� �� ����
                }
                else { // ȸ���� �ʿ� ���� ��� ��� �̵�
                    agent->pos = next_pos[i];
                    agent->facing_direction = required_dir;
                    agent->total_distance_traveled += 1.0;
                }
            }
            else { // �̵��� ���� ��� (��ǥ ���� or ���)
                agent->facing_direction = STATIONARY;
            }
        }

        // 5. �̵� �� ��ǥ ���� �� ���� ������Ʈ
        agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->logger);

        // 6. ���� ���¸� ȭ�鿡 ǥ��
        simulation_display_status(sim);

        // 7. �ùķ��̼� �Ϸ� ���� Ȯ��
        if (simulation_is_complete(sim)) break;

        sim->scenario_manager->time_step++;
        sleep(sim->scenario_manager->simulation_speed);
    }
}


// =============================================================================
// --- 11. ���� �Լ� ---
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
        printf("\n�ùķ��̼��� ��ҵǾ����ϴ�. ���α׷��� �����մϴ�.\n");
    }

    simulation_destroy(sim);
    return 0;
}