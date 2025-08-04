/**
* @file d_lite_parking.c
* @brief D* Lite 알고리즘 기반 3-에이전트 자동 주차 시뮬레이션 (회전 딜레이 및 작업 딜레이 적용)
*/
#define _CRT_SECURE_NO_WARNINGS // visual studio 환경에서 보안 경고를 비활성화하기 위한 매크로
#include <stdio.h>      // 표준 입출력 함수(printf, scanf 등)를 사용하기 위한 헤더 파일
#include <stdlib.h>     // 동적 메모리 할당(malloc, free), 난수 생성(rand) 등을 위한 헤더 파일
#include <string.h>     // 문자열 처리 함수(strcpy, strcmp 등)를 사용하기 위한 헤더 파일
#include <math.h>       // 수학 함수(sqrt, fabs 등)를 사용하기 위한 헤더 파일
#include <float.h>      // 부동소수점 수의 최대값(DBL_MAX) 등 한계값을 정의한 헤더 파일
#include <time.h>       // 시간 관련 함수(time)를 사용하기 위한 헤더 파일 (난수 시드 생성에 사용)
#include <stdarg.h>     // 가변 인자 함수(vsnprintf 등)를 처리하기 위한 헤더 파일
#include <ctype.h>      // 문자 처리 함수(tolower)를 사용하기 위한 헤더 파일

#ifdef _WIN32 // 운영체제가 windows일 경우
#include <windows.h>    // windows api 함수(Sleep, SetConsoleCursorPosition 등)를 사용하기 위한 헤더 파일
#include <conio.h>      // 콘솔 입출력 함수(_getch)를 사용하기 위한 헤더 파일
#define sleep(ms) Sleep(ms) // Sleep 함수를 밀리초 단위로 정의
#else // windows가 아닌 다른 운영체제(linux, macos 등)일 경우
#include <unistd.h>     // posix 운영체제 api(usleep, read)를 사용하기 위한 헤더 파일
#include <termios.h>    // 터미널 제어(에코 비활성화 등)를 위한 헤더 파일
#define sleep(ms) usleep(ms * 1000) // usleep 함수를 밀리초 단위로 정의 (usleep은 마이크로초 단위)
#endif

// =============================================================================
// --- 1. 상수 및 전역 설정 정의 ---
// =============================================================================
// --- 시스템 상수 ---
#define TRUE 1                  // 불리언 참(true) 값 정의
#define FALSE 0                 // 불리언 거짓(false) 값 정의
#define INPUT_BUFFER_SIZE 100   // 사용자 입력 버퍼의 크기 정의
#define DISPLAY_BUFFER_SIZE 16384 // 화면 렌더링을 위한 버퍼 크기 정의 (성능 최적화용)

// --- ANSI 컬러 코드 ---
#define C_NRM "\x1b[0m"       // 기본 색상으로 리셋
#define C_RED "\x1b[31m"       // 빨간색
#define C_GRN "\x1b[32m"       // 초록색
#define C_YEL "\x1b[33m"       // 노란색
#define C_BLU "\x1b[34m"       // 파란색
#define C_MAG "\x1b[35m"       // 마젠타색
#define C_CYN "\x1b[36m"       // 시안색
#define C_WHT "\x1b[37m"       // 흰색
#define C_GRY "\x1b[90m"       // 회색
#define C_B_RED "\x1b[1;31m"   // 밝은 빨간색 (굵게)
#define C_B_GRN "\x1b[1;32m"   // 밝은 초록색 (굵게)
#define C_B_YEL "\x1b[1;33m"   // 밝은 노란색 (굵게)
#define C_B_MAG "\x1b[1;35m"   // 밝은 마젠타색 (굵게)
#define C_B_CYN "\x1b[1;36m"   // 밝은 시안색 (굵게)
#define C_B_WHT "\x1b[1;37m"   // 밝은 흰색 (굵게)

// --- 시뮬레이션 그리드 및 에이전트 상수 ---
#define GRID_WIDTH 37           // 그리드 맵의 너비
#define GRID_HEIGHT 12          // 그리드 맵의 높이
#define MAX_AGENTS 3            // 시뮬레이션에 참여하는 최대 에이전트(로봇) 수
#define MAX_GOALS (GRID_WIDTH * GRID_HEIGHT) // 가능한 최대 목표 지점(주차 공간) 수
#define INF DBL_MAX             // 무한대 값을 double의 최대값으로 정의 (비용 계산에 사용)
#define NUM_DIRECTIONS 4        // 에이전트가 이동할 수 있는 방향의 수 (상, 하, 좌, 우)

// --- 시뮬레이션 동작 상수 ---
#define DISTANCE_BEFORE_CHARGE 300.0 // 충전이 필요해지는 기준 이동 거리
#define CHARGE_TIME 20               // 충전에 걸리는 시간 (시뮬레이션 스텝 기준)
#define PARKING_TIME 5               // 주차 작업에 소요되는 시간 (시뮬레이션 스텝 기준)
#define COLLECTION_TIME 5            // 차량 수거 작업에 소요되는 시간 (시뮬레이션 스텝 기준)
#define ROTATION_TIME 2              // [ADDED] 회전에 소요되는 시간 (시뮬레이션 스텝 기준)
#define MAX_CHARGE_STATIONS 10       // 최대 충전소 개수
#define MAX_PHASES 20                // 사용자 정의 시나리오에서 설정 가능한 최대 단계 수
#define REALTIME_MODE_TIMELIMIT 10000 // 실시간 모드의 최대 시뮬레이션 시간
#define MAX_TASKS 50                 // 작업 큐에 저장할 수 있는 최대 작업 수
#define MAX_SPEED_MULTIPLIER 100.0f  // 시뮬레이션 최대 배속
#define EVENT_GENERATION_INTERVAL 5 // 실시간 모드에서 이벤트(주차/출차 요청)가 발생하는 시간 간격

// --- 로깅 및 UI 상수 ---
#define LOG_BUFFER_LINES 5      // 화면에 표시할 로그 메시지의 최대 줄 수
#define LOG_BUFFER_WIDTH 256    // 각 로그 메시지의 최대 길이
#define STATUS_STRING_WIDTH 25  // 에이전트 상태 문자열을 출력할 때의 고정 너비 (UI 정렬용)

// =============================================================================
// --- 2. 구조체 정의 ---
// =============================================================================
// --- 기본 구조체 ---
/** @struct Key
 * @brief 우선순위 큐(priority queue)에서 노드의 우선순위를 결정하는 키 값.
 */
typedef struct { double k1; double k2; } Key;

/** @struct Node
 * @brief 그리드 맵의 각 셀(노드)을 나타내는 구조체.
 */
typedef struct Node {
    int x, y;                   // 노드의 그리드 상 x, y 좌표
    double g, rhs;              // D* Lite 알고리즘에서 사용하는 g값(실제 비용)과 rhs값(예상 비용)
    Key key;                    // 우선순위 큐에서의 정렬을 위한 키 값
    int is_obstacle;            // 이 노드가 장애물인지 여부 (TRUE/FALSE)
    int is_goal;                // 이 노드가 주차 공간(목표)인지 여부
    int is_temp;                // 임시 장애물로 설정되었는지 여부 (다른 에이전트 회피용)
    int is_parked;              // 이 노드에 차량이 주차되어 있는지 여부
    int reserved_by_agent;      // 어떤 에이전트에 의해 예약되었는지 (에이전트 ID, -1은 예약 없음)
    int in_pq;                  // 현재 우선순위 큐에 포함되어 있는지 여부
    int pq_index;               // 우선순위 큐(힙) 내에서의 인덱스
} Node;

/** @enum AgentState
 * @brief 에이전트의 현재 상태를 나타내는 열거형.
 */
typedef enum {
    IDLE,                       // 대기 상태
    ROTATING,                   // [ADDED] 회전 중
    GOING_TO_PARK,              // 주차하러 가는 중
    PERFORMING_PARKING,         // 주차 작업 수행 중
    RETURNING_HOME_EMPTY,       // 주차 후 빈 몸으로 기지로 복귀하는 중
    GOING_TO_COLLECT,           // 출차할 차를 가지러 가는 중
    PERFORMING_COLLECTION,      // 차량 수거 작업 수행 중
    RETURNING_WITH_CAR,         // 차를 싣고 출차 지점으로 가는 중
    GOING_TO_CHARGE,            // 충전소로 가는 중
    CHARGING,                   // 충전 중
    RETURNING_HOME_MAINTENANCE  // 충전 후 기지로 복귀하는 상태
} AgentState;

/** @enum AgentDirection
 * @brief [ADDED] 에이전트가 바라보는 방향을 나타내는 열거형.
 */
typedef enum {
    NORTH,      // 북쪽 (y-1)
    EAST,       // 동쪽 (x+1)
    SOUTH,      // 남쪽 (y+1)
    WEST,       // 서쪽 (x-1)
    STATIONARY  // 정지 상태
} AgentDirection;

/** @struct Agent
 * @brief 하나의 에이전트(로봇)를 나타내는 구조체.
 */
typedef struct Agent {
    int id;                     // 에이전트의 고유 ID (0, 1, 2)
    char symbol;                // 맵에 표시될 에이전트의 기호 ('A', 'B', 'C')
    Node* pos;                  // 현재 위치를 가리키는 노드 포인터
    Node* home_base;            // 에이전트의 기지(시작점)를 가리키는 노드
    Node* goal;                 // 현재 목표 지점을 가리키는 노드 포인터
    AgentState state;           // 에이전트의 현재 상태
    AgentState previous_state;  // [ADDED] 회전 등 임시 상태 이전에 가지고 있던 상태
    AgentDirection facing_direction; // [ADDED] 현재 바라보는 방향
    Node* intended_next_pos;    // [ADDED] 회전 후 이동할 다음 위치
    double total_distance_traveled; // 총 이동 거리 (충전 필요 여부 판단에 사용)
    int charge_timer;           // 충전 잔여 시간
    int action_timer;           // 주차/수거/회전 작업 잔여 시간
} Agent;

/** @struct PriorityQueue
 * @brief 최소 힙(min-heap)으로 구현된 우선순위 큐.
 */
typedef struct {
    Node** nodes;   // 노드 포인터를 저장하는 배열
    int size;       // 현재 큐에 저장된 요소의 개수
    int capacity;   // 큐의 최대 용량
} PriorityQueue;

/** @enum PhaseType
 * @brief 사용자 정의 시나리오의 각 단계 유형을 나타내는 열거형.
 */
typedef enum { PARK_PHASE, EXIT_PHASE } PhaseType;

/** @struct DynamicPhase
 * @brief 사용자 정의 시나리오의 한 단계를 정의하는 구조체.
 */
typedef struct {
    PhaseType type;             // 단계의 유형 (주차 또는 출차)
    int task_count;             // 이 단계에서 수행해야 할 작업(차량)의 수
    char type_name[10];         // 단계 유형을 문자열로 저장 (UI 표시용)
} DynamicPhase;

/** @enum TaskType
 * @brief 실시간 모드에서 생성되는 작업의 유형을 나타내는 열거형.
 */
typedef enum { TASK_NONE, TASK_PARK, TASK_EXIT } TaskType;

/** @struct TaskNode
 * @brief 작업 큐의 단일 노드를 나타내는 연결 리스트 구조체.
 */
typedef struct TaskNode {
    TaskType type;
    struct TaskNode* next;
} TaskNode;

/** @enum SimulationMode
 * @brief 시뮬레이션의 전체 동작 모드를 나타내는 열거형.
 */
typedef enum { MODE_UNINITIALIZED, MODE_CUSTOM, MODE_REALTIME } SimulationMode;

// --- 모듈화된 구조체 ---
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
// --- 3. 함수 프로토타입 선언 ---
// =============================================================================
// --- 모듈 생성/소멸 함수 ---
Simulation* simulation_create();
void simulation_destroy(Simulation* sim);

// --- 시스템, UI 및 로깅 함수 ---
void system_enable_virtual_terminal();
void ui_clear_screen_optimized();
int simulation_setup(Simulation* sim);
void logger_log(Logger* logger, const char* format, ...);

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
Pathfinder* pathfinder_create(Node* start, Node* goal);
void pathfinder_destroy(Pathfinder* pf);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, AgentManager* agent_manager);
Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager, Node* current_node);

// --- 에이전트 (AgentManager) 로직 함수 ---
void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]);
void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger);
void agent_manager_update_charge_state(AgentManager* manager, Logger* logger);
void agent_manager_update_action_state(AgentManager* manager, Logger* logger, GridMap* map); // [MODIFIED] map 전달

// --- 시뮬레이션 (Simulation) 관리 함수 ---
void simulation_run(Simulation* sim);

// =============================================================================
// --- 4. 시스템, UI 및 로깅 구현 ---
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
        printf(C_B_RED "\n잘못된 입력입니다. 유효한 키를 눌러주세요. (%s)\n" C_NRM, valid_chars);
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
        printf(C_B_RED "잘못된 입력입니다. %d에서 %d 사이의 정수를 입력하세요.\n" C_NRM, min, max);
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
        printf(C_B_RED "잘못된 입력입니다. %.1f에서 %.1f 사이의 숫자를 입력하세요.\n" C_NRM, min, max);
    }
}

static int simulation_setup_custom_scenario(ScenarioManager* scenario) {
    printf(C_B_WHT "--- 사용자 정의 시나리오 설정 ---\n" C_NRM);
    scenario->num_phases = get_integer_input(C_YEL "총 단계 수를 입력하세요 (1-20, 0=취소): " C_NRM, 0, MAX_PHASES);
    if (scenario->num_phases == 0) return 0;

    for (int i = 0; i < scenario->num_phases; i++) {
        printf(C_B_CYN "\n--- %d/%d 단계 설정 ---\n" C_NRM, i + 1, scenario->num_phases);
        printf("a. %s주차%s\n", C_YEL, C_NRM);
        printf("b. %s출차%s\n", C_CYN, C_NRM);
        char type_choice = get_char_input("단계 유형을 선택하세요: ", "ab");
        scenario->phases[i].task_count = get_integer_input("이 단계에서 처리할 차량 수를 입력하세요: ", 1, 100);

        if (type_choice == 'a') {
            scenario->phases[i].type = PARK_PHASE;
            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "주차");
        }
        else {
            scenario->phases[i].type = EXIT_PHASE;
            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "출차");
        }
        printf(C_GRN "%d단계 설정 완료: %s %d대.\n" C_NRM, i + 1, scenario->phases[i].type_name, scenario->phases[i].task_count);
    }
    printf(C_B_GRN "\n--- 시나리오 설정이 완료되었습니다! ---\n" C_NRM);
    sleep(1500);
    return 1;
}

static int simulation_setup_realtime(ScenarioManager* scenario) {
    printf(C_B_WHT "--- 실시간 시뮬레이션 설정 ---\n" C_NRM);
    while (TRUE) {
        scenario->park_chance = get_integer_input("\n주차 요청 발생 확률(0~100)을 입력하세요: ", 0, 100);
        scenario->exit_chance = get_integer_input("출차 요청 발생 확률(0~100)을 입력하세요: ", 0, 100);
        if ((scenario->park_chance + scenario->exit_chance) <= 100) break;
        printf(C_B_RED "주차와 출차 확률의 합은 100을 넘을 수 없습니다.\n" C_NRM);
    }
    printf(C_B_GRN "\n설정 완료: 주차 확률 %d%%, 출차 확률 %d%%\n" C_NRM, scenario->park_chance, scenario->exit_chance);
    sleep(1500);
    return 1;
}

static int simulation_setup_speed(ScenarioManager* scenario) {
    printf(C_B_WHT "\n--- 시뮬레이션 속도 설정 ---\n" C_NRM);
    scenario->speed_multiplier = get_float_input("원하는 배속을 입력하세요 (1.0 ~ 100.0): ", 1.0f, MAX_SPEED_MULTIPLIER);
    scenario->simulation_speed = (int)(100.0f / scenario->speed_multiplier);
    if (scenario->simulation_speed < 1) scenario->simulation_speed = 1;
    printf(C_B_GRN "\n--- 시뮬레이션을 %.1fx 배속으로 시작합니다... ---\n" C_NRM, scenario->speed_multiplier);
    sleep(1500);
    return 1;
}

int simulation_setup(Simulation* sim) {
    ui_clear_screen_optimized();
    printf(C_B_WHT "--- 시뮬레이션 모드 선택 ---\n" C_NRM);
    printf("a. %s사용자 정의 시나리오%s\n", C_YEL, C_NRM);
    printf("b. %s실시간 시뮬레이션%s\n", C_CYN, C_NRM);
    printf("q. %s종료%s\n\n", C_RED, C_NRM);

    char choice = get_char_input("실행할 시나리오 문자를 입력하세요: ", "abq");
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

    // [MODIFIED] 상태, 색상, 방향 문자열 추가
    const char* agent_state_strings[] = { "대기", "회전 중", "주차 공간으로 이동", "주차 작업 중", "기지 복귀(빈 차)", "수거하러 이동", "수거 작업 중", "출차 중(차량 탑승)", "충전소로 이동", "충전 중", "기지 복귀(충전 후)" };
    const char* agent_state_colors[] = { C_GRY, C_B_CYN, C_YEL, C_B_YEL, C_CYN, C_YEL, C_B_YEL, C_GRN, C_B_RED, C_RED, C_CYN };
    const char* direction_symbols[] = { "^", ">", "v", "<", " " }; // 북, 동, 남, 서, 정지

    for (int i = 0; i < MAX_AGENTS; i++) {
        const Agent* agent = &agent_manager->agents[i];
        const char* agent_color = (i == 0) ? C_B_CYN : (i == 1) ? C_B_YEL : C_B_MAG;
        char status_buffer[100];

        // [MODIFIED] 회전 상태 타이머 표시 로직 추가
        if (agent->state == CHARGING) {
            snprintf(status_buffer, sizeof(status_buffer), "충전 중... (%d)", agent->charge_timer);
        }
        else if (agent->state == PERFORMING_PARKING) {
            snprintf(status_buffer, sizeof(status_buffer), "주차 작업 중... (%d)", agent->action_timer);
        }
        else if (agent->state == PERFORMING_COLLECTION) {
            snprintf(status_buffer, sizeof(status_buffer), "수거 작업 중... (%d)", agent->action_timer);
        }
        else if (agent->state == ROTATING) {
            snprintf(status_buffer, sizeof(status_buffer), "회전 중... (%d)", agent->action_timer);
        }
        else {
            snprintf(status_buffer, sizeof(status_buffer), "%s", agent_state_strings[agent->state]);
        }

        // [MODIFIED] 에이전트 방향 표시 추가
        written = snprintf(buf_ptr, remaining_size, "%sAgent %c%s (%s%s%s): (%2d,%d) ", agent_color, agent->symbol, C_NRM, C_B_WHT, direction_symbols[agent->facing_direction], C_NRM, agent->pos->x, agent->pos->y);
        buf_ptr += written; remaining_size -= written;

        if (agent->goal) {
            written = snprintf(buf_ptr, remaining_size, "-> (%2d,%d) ", agent->goal->x, agent->goal->y);
        }
        else {
            written = snprintf(buf_ptr, remaining_size, "-> 없음    ");
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
    if (!logger) { perror("Logger 할당 실패"); exit(1); }
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

// ... (Priority Queue 구현부는 변경 사항이 없으므로 생략) ...
// =============================================================================
// --- 5. 유틸리티 구현 (우선순위 큐) ---
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
// --- 6. 맵 관리 구현 (GridMap) ---
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
                fprintf(stderr, C_B_RED "오류: 내장된 맵 데이터가 예상보다 짧습니다.\n" C_NRM);
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
        fprintf(stderr, C_B_RED "오류: 맵 데이터에 충전소('e')가 없습니다.\n" C_NRM);
        exit(1);
    }
}

GridMap* grid_map_create(AgentManager* agent_manager) {
    GridMap* map = (GridMap*)calloc(1, sizeof(GridMap));
    if (!map) { perror("GridMap 할당 실패"); exit(1); }
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
                agent_manager->agents[i].state == ROTATING)) { // [MODIFIED] 회전 중에도 막힌 것으로 간주
            return TRUE;
        }
    }
    return FALSE;
}

// ... (ScenarioManager, Pathfinder 구현부는 변경 사항 없음) ...
// =============================================================================
// --- 7. 시나리오 및 작업 관리 구현 (ScenarioManager) ---
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
    if (!manager) { perror("ScenarioManager 할당 실패"); exit(1); }
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
// --- 8. 경로 탐색 구현 (Pathfinder - D* Lite) ---
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
// --- 9. 에이전트 관리 및 로직 구현 (AgentManager) ---
// =============================================================================
AgentManager* agent_manager_create() {
    AgentManager* manager = (AgentManager*)calloc(1, sizeof(AgentManager));
    if (!manager) { perror("AgentManager 할당 실패"); exit(1); }
    for (int i = 0; i < MAX_AGENTS; i++) {
        manager->agents[i].id = i;
        manager->agents[i].symbol = 'A' + i;
        manager->agents[i].state = IDLE;
        manager->agents[i].action_timer = 0;
        manager->agents[i].facing_direction = STATIONARY; // [ADDED] 방향 초기화
        manager->agents[i].intended_next_pos = NULL;      // [ADDED] 의도 위치 초기화
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
        logger_log(logger, "[%sPlan%s] Agent %c, 주차 공간 (%d,%d) 선택 (비용: %.1f)", C_CYN, C_NRM, agent->symbol, best_goal->x, best_goal->y, best_cost);
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
        logger_log(logger, "[%sPlan%s] Agent %c, 출차 차량 (%d,%d) 선택 (비용: %.1f)", C_CYN, C_NRM, agent->symbol, best_spot->x, best_spot->y, best_cost);
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
        logger_log(logger, "[%sPlan%s] Agent %c, 충전소 (%d,%d) 선택 (비용: %.1f)", C_CYN, C_NRM, agent->symbol, best_station->x, best_station->y, best_cost);
    }
    return best_station;
}

static void agent_set_goal(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
    if (agent->state == RETURNING_HOME_EMPTY && agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
        if (agent->goal) {
            agent->goal->reserved_by_agent = -1;
            agent->goal = NULL;
        }
        logger_log(logger, "[%sCharge%s] Agent %c 충전 필요! 목표를 충전소로 즉시 변경.", C_B_YEL, C_NRM, agent->symbol);
        agent->state = GOING_TO_CHARGE;
    }

    // [MODIFIED] 회전 중이거나 다른 작업 중일 때 목표 재설정 방지
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
        logger_log(logger, "[%sInfo%s] Agent %c: 가용 목표 없음. 대기 상태로 전환.", C_YEL, C_NRM, agent->symbol);
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
                logger_log(logger, "[%sAvoid%s] 충돌 감지! Agent %c 대기.", C_B_RED, C_NRM, manager->agents[j].symbol);
                next_pos[j] = manager->agents[j].pos;
            }
            else if (next_pos[i] == manager->agents[j].pos && next_pos[j] == manager->agents[i].pos) {
                logger_log(logger, "[%sAvoid%s] 교차 충돌 감지! Agent %c 대기.", C_B_RED, C_NRM, manager->agents[j].symbol);
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
            logger_log(logger, "[%sPark%s] Agent %c, 주차 작업 시작 at (%d,%d)... (%d steps)", C_B_YEL, C_NRM, agent->symbol, reached_goal->x, reached_goal->y, PARKING_TIME);
            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == PARK_PHASE) {
                scenario->tasks_completed_in_phase++;
            }
            agent->state = PERFORMING_PARKING;
            agent->action_timer = PARKING_TIME;
            break;

        case RETURNING_HOME_EMPTY:
            logger_log(logger, "[%sInfo%s] Agent %c, 주차 작업 후 기지 복귀 완료.", C_CYN, C_NRM, agent->symbol);
            agent->state = IDLE;
            agent->facing_direction = STATIONARY;
            reached_goal->reserved_by_agent = -1;
            break;

        case GOING_TO_COLLECT:
            reached_goal->is_parked = FALSE;
            manager->total_cars_parked--;
            logger_log(logger, "[%sExit%s] Agent %c, 차량 수거 작업 시작 at (%d,%d)... (%d steps)", C_B_YEL, C_NRM, agent->symbol, reached_goal->x, reached_goal->y, COLLECTION_TIME);
            agent->state = PERFORMING_COLLECTION;
            agent->action_timer = COLLECTION_TIME;
            break;

        case RETURNING_WITH_CAR:
            logger_log(logger, "[%sExit%s] Agent %c, 차량 출차 완료.", C_GRN, C_NRM, agent->symbol);
            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == EXIT_PHASE) {
                scenario->tasks_completed_in_phase++;
            }
            agent->state = IDLE;
            agent->facing_direction = STATIONARY;
            reached_goal->reserved_by_agent = -1;
            break;

        case GOING_TO_CHARGE:
            logger_log(logger, "[%sCharge%s] Agent %c, 충전 시작. (%d steps)", C_B_YEL, C_NRM, agent->symbol, CHARGE_TIME);
            agent->state = CHARGING;
            agent->charge_timer = CHARGE_TIME;
            break;

        case RETURNING_HOME_MAINTENANCE:
            logger_log(logger, "[%sInfo%s] Agent %c, 충전 후 기지 복귀 완료.", C_CYN, C_NRM, agent->symbol);
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
                logger_log(logger, "[%sCharge%s] Agent %c 충전 완료.", C_B_GRN, C_NRM, agent->symbol);
                agent->total_distance_traveled = 0.0;
                agent->state = RETURNING_HOME_MAINTENANCE;
                if (agent->pos) agent->pos->reserved_by_agent = -1;
                agent->goal = NULL;
            }
        }
    }
}

// [ADDED] 현재 위치와 다음 위치를 기반으로 필요한 방향을 계산하는 헬퍼 함수
static AgentDirection calculate_required_direction(Node* current, Node* next) {
    if (next->y < current->y) return NORTH;
    if (next->y > current->y) return SOUTH;
    if (next->x < current->x) return WEST;
    if (next->x > current->x) return EAST;
    return STATIONARY;
}


/**
 * @brief [MODIFIED] 주차, 수거, 회전 작업 중인 에이전트의 상태를 업데이트합니다.
 */
void agent_manager_update_action_state(AgentManager* manager, Logger* logger, GridMap* map) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (agent->state == PERFORMING_PARKING) {
            agent->action_timer--;
            if (agent->action_timer <= 0) {
                logger_log(logger, "[%sPark%s] Agent %c, 주차 작업 완료. 기지로 복귀합니다.", C_GRN, C_NRM, agent->symbol);
                agent->state = RETURNING_HOME_EMPTY;
                agent->goal = NULL;
            }
        }
        else if (agent->state == PERFORMING_COLLECTION) {
            agent->action_timer--;
            if (agent->action_timer <= 0) {
                logger_log(logger, "[%sExit%s] Agent %c, 수거 작업 완료. 출차 지점으로 이동합니다.", C_GRN, C_NRM, agent->symbol);
                agent->state = RETURNING_WITH_CAR;
                agent->goal = NULL;
            }
        }
        else if (agent->state == ROTATING) { // [ADDED] 회전 상태 처리
            agent->action_timer--;
            if (agent->action_timer <= 0) {
                AgentDirection required_dir = calculate_required_direction(agent->pos, agent->intended_next_pos);
                logger_log(logger, "[%sMove%s] Agent %c, 회전 완료. 이동 재개.", C_CYN, C_NRM, agent->symbol);

                // 충돌 재확인: 내가 회전하는 동안 누군가 내 목적지를 막았을 수 있음
                if (grid_is_node_blocked(map, manager, agent->intended_next_pos)) {
                    logger_log(logger, "[%sAvoid%s] Agent %c, 회전 후 이동하려 했으나 경로 막힘. 대기.", C_B_RED, C_NRM, agent->symbol);
                    agent->state = agent->previous_state; // 원래 상태로 복귀
                    agent->facing_direction = required_dir; // 방향은 변경
                    agent->intended_next_pos = NULL;
                }
                else {
                    agent->state = agent->previous_state; // 원래 상태로 복귀
                    agent->facing_direction = required_dir;
                    agent->pos = agent->intended_next_pos; // 저장해둔 위치로 이동
                    agent->total_distance_traveled += 1.0;
                    agent->intended_next_pos = NULL;
                }
            }
        }
    }
}


// =============================================================================
// --- 10. 시뮬레이션 코어 로직 구현 ---
// =============================================================================
Simulation* simulation_create() {
    Simulation* sim = (Simulation*)calloc(1, sizeof(Simulation));
    if (!sim) { perror("Simulation 할당 실패"); exit(1); }
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
    // ... (이 함수는 변경 사항 없음, 내용은 위와 동일) ...
    ScenarioManager* scenario = sim->scenario_manager;
    AgentManager* agent_manager = sim->agent_manager;
    GridMap* map = sim->map;
    Logger* logger = sim->logger;

    if (scenario->mode == MODE_CUSTOM) {
        if (scenario->current_phase_index >= scenario->num_phases) return;
        DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
        if (scenario->tasks_completed_in_phase >= phase->task_count) {
            logger_log(logger, "[%sPhase%s] %d단계 (%s %d대) 완료!", C_B_YEL, C_NRM, scenario->current_phase_index + 1, phase->type_name, phase->task_count);
            scenario->current_phase_index++;
            scenario->tasks_completed_in_phase = 0;
            if (scenario->current_phase_index < scenario->num_phases) {
                DynamicPhase* next_phase = &scenario->phases[scenario->current_phase_index];
                logger_log(logger, "[%sPhase%s] %d단계 시작: %s %d대.", C_B_YEL, C_NRM, scenario->current_phase_index + 1, next_phase->type_name, next_phase->task_count);
                sleep(1500);
            }
            return;
        }
    }
    else if (scenario->mode == MODE_REALTIME) {
        if (scenario->time_step > 0 && scenario->time_step % EVENT_GENERATION_INTERVAL == 0) {
            int event_chance = rand() % 100;
            if (event_chance < scenario->park_chance && agent_manager->total_cars_parked < map->num_goals) {
                logger_log(logger, "[%sEvent%s] 새로운 주차 요청 발생.", C_B_GRN, C_NRM);
                add_task_to_queue(scenario, TASK_PARK);
            }
            else if (event_chance < (scenario->park_chance + scenario->exit_chance) && agent_manager->total_cars_parked > 0) {
                logger_log(logger, "[%sEvent%s] 새로운 출차 요청 발생.", C_B_YEL, C_NRM);
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
                    logger_log(logger, "[%sWarn%s] Agent %c 충전 필요하나 모든 충전소가 사용 중.", C_YEL, C_NRM, agent->symbol);
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
                        logger_log(logger, "[%sTask%s] Agent %c, 신규 주차 작업 할당.", C_CYN, C_NRM, agent->symbol);
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
                        logger_log(logger, "[%sTask%s] Agent %c, 신규 출차 작업 할당.", C_CYN, C_NRM, agent->symbol);
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
                            logger_log(logger, "[%sTask%s] Agent %c, 주차 작업 할당.", C_CYN, C_NRM, agent->symbol);
                        }
                        else {
                            agent->state = GOING_TO_COLLECT;
                            logger_log(logger, "[%sTask%s] Agent %c, 출차 작업 할당.", C_CYN, C_NRM, agent->symbol);
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
        printf(C_B_GRN "\n모든 시나리오 단계가 완료되었습니다! 시뮬레이션을 종료합니다.\n" C_NRM);
        return TRUE;
    }

    if (scenario->mode == MODE_REALTIME && scenario->time_step >= REALTIME_MODE_TIMELIMIT) {
        printf(C_B_GRN "\n실시간 시뮬레이션 시간 제한에 도달했습니다! 시뮬레이션을 종료합니다.\n" C_NRM);
        return TRUE;
    }
    return FALSE;
}

void simulation_run(Simulation* sim) {
    while (TRUE) { // 무한 루프
        // 1. 타이머 기반 상태 업데이트
        agent_manager_update_charge_state(sim->agent_manager, sim->logger);
        agent_manager_update_action_state(sim->agent_manager, sim->logger, sim->map);

        // 2. 작업 할당
        simulation_update_state(sim);

        // 3. 경로 계획 및 충돌 회피
        Node* next_pos[MAX_AGENTS];
        agent_manager_plan_and_resolve_collisions(sim->agent_manager, sim->map, sim->logger, next_pos);

        // 4. [MODIFIED] 회전 감지 및 이동 실행 로직
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* agent = &sim->agent_manager->agents[i];
            AgentState s = agent->state;

            // 타이머가 동작 중인 상태(충전, 작업, 회전)에서는 새로운 이동을 시작하지 않음
            if (s == CHARGING || s == PERFORMING_PARKING || s == PERFORMING_COLLECTION || s == ROTATING) {
                continue;
            }

            if (agent->pos != next_pos[i] && next_pos[i] != NULL) { // 이동이 필요한 경우
                AgentDirection required_dir = calculate_required_direction(agent->pos, next_pos[i]);

                // 회전이 필요한지 확인 (정지 상태에서 처음 움직이는 경우는 제외)
                if (agent->facing_direction != required_dir && agent->facing_direction != STATIONARY) {
                    // [FIXED] logger를 sim->logger로 수정하여 오류 해결
                    logger_log(sim->logger, "[%sMove%s] Agent %c, 회전 필요. (%d steps)", C_YEL, C_NRM, agent->symbol, ROTATION_TIME);
                    agent->previous_state = agent->state; // 현재 상태 저장
                    agent->state = ROTATING;              // 회전 상태로 변경
                    agent->action_timer = ROTATION_TIME;  // 회전 타이머 설정
                    agent->intended_next_pos = next_pos[i]; // 회전 후 갈 곳 저장
                }
                else { // 회전이 필요 없는 경우 즉시 이동
                    agent->pos = next_pos[i];
                    agent->facing_direction = required_dir;
                    agent->total_distance_traveled += 1.0;
                }
            }
            else { // 이동이 없는 경우 (목표 도달 or 대기)
                agent->facing_direction = STATIONARY;
            }
        }

        // 5. 이동 후 목표 도달 등 상태 업데이트
        agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->logger);

        // 6. 현재 상태를 화면에 표시
        simulation_display_status(sim);

        // 7. 시뮬레이션 완료 조건 확인
        if (simulation_is_complete(sim)) break;

        sim->scenario_manager->time_step++;
        sleep(sim->scenario_manager->simulation_speed);
    }
}


// =============================================================================
// --- 11. 메인 함수 ---
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
        printf("\n시뮬레이션이 취소되었습니다. 프로그램을 종료합니다.\n");
    }

    simulation_destroy(sim);
    return 0;
}