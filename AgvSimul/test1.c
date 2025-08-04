///**
//* @file lpa_parking.c
//* @brief lpa* 알고리즘 기반 3-에이전트 자동 주차 시뮬레이션 (연결 리스트 큐 및 개선된 작업 처리 로직 적용)
//*/
//#define _CRT_SECURE_NO_WARNINGS // visual studio 환경에서 보안 경고를 비활성화하기 위한 매크로
//#include <stdio.h>      // 표준 입출력 함수(printf, scanf 등)를 사용하기 위한 헤더 파일
//#include <stdlib.h>     // 동적 메모리 할당(malloc, free), 난수 생성(rand) 등을 위한 헤더 파일
//#include <string.h>     // 문자열 처리 함수(strcpy, strcmp 등)를 사용하기 위한 헤더 파일
//#include <math.h>       // 수학 함수(sqrt, fabs 등)를 사용하기 위한 헤더 파일
//#include <float.h>      // 부동소수점 수의 최대값(DBL_MAX) 등 한계값을 정의한 헤더 파일
//#include <time.h>       // 시간 관련 함수(time)를 사용하기 위한 헤더 파일 (난수 시드 생성에 사용)
//#include <stdarg.h>     // 가변 인자 함수(vsnprintf 등)를 처리하기 위한 헤더 파일
//#include <ctype.h>      // 문자 처리 함수(tolower)를 사용하기 위한 헤더 파일
//
//#ifdef _WIN32 // 운영체제가 windows일 경우
//#include <windows.h>    // windows api 함수(Sleep, SetConsoleCursorPosition 등)를 사용하기 위한 헤더 파일
//#include <conio.h>      // 콘솔 입출력 함수(_getch)를 사용하기 위한 헤더 파일
//#define sleep(ms) Sleep(ms) // Sleep 함수를 밀리초 단위로 정의
//#else // windows가 아닌 다른 운영체제(linux, macos 등)일 경우
//#include <unistd.h>     // posix 운영체제 api(usleep, read)를 사용하기 위한 헤더 파일
//#include <termios.h>    // 터미널 제어(에코 비활성화 등)를 위한 헤더 파일
//#define sleep(ms) usleep(ms * 1000) // usleep 함수를 밀리초 단위로 정의 (usleep은 마이크로초 단위)
//#endif
//
//// =============================================================================
//// --- 1. 상수 및 전역 설정 정의 ---
//// =============================================================================
//// --- 시스템 상수 ---
//#define TRUE 1                  // 불리언 참(true) 값 정의
//#define FALSE 0                 // 불리언 거짓(false) 값 정의
//#define INPUT_BUFFER_SIZE 100   // 사용자 입력 버퍼의 크기 정의
//#define DISPLAY_BUFFER_SIZE 16384 // 화면 렌더링을 위한 버퍼 크기 정의 (성능 최적화용)
//
//// --- ANSI 컬러 코드 ---
//// 터미널 텍스트 색상을 제어하기 위한 ANSI 이스케이프 코드 정의
//#define C_NRM "\x1b[0m"       // 기본 색상으로 리셋
//#define C_RED "\x1b[31m"       // 빨간색
//#define C_GRN "\x1b[32m"       // 초록색
//#define C_YEL "\x1b[33m"       // 노란색
//#define C_BLU "\x1b[34m"       // 파란색
//#define C_MAG "\x1b[35m"       // 마젠타색
//#define C_CYN "\x1b[36m"       // 시안색
//#define C_WHT "\x1b[37m"       // 흰색
//#define C_GRY "\x1b[90m"       // 회색
//#define C_B_RED "\x1b[1;31m"   // 밝은 빨간색 (굵게)
//#define C_B_GRN "\x1b[1;32m"   // 밝은 초록색 (굵게)
//#define C_B_YEL "\x1b[1;33m"   // 밝은 노란색 (굵게)
//#define C_B_MAG "\x1b[1;35m"   // 밝은 마젠타색 (굵게)
//#define C_B_CYN "\x1b[1;36m"   // 밝은 시안색 (굵게)
//#define C_B_WHT "\x1b[1;37m"   // 밝은 흰색 (굵게)
//
//// --- 시뮬레이션 그리드 및 에이전트 상수 ---
//#define GRID_WIDTH 37           // 그리드 맵의 너비
//#define GRID_HEIGHT 12          // 그리드 맵의 높이
//#define MAX_AGENTS 3            // 시뮬레이션에 참여하는 최대 에이전트(로봇) 수
//#define MAX_GOALS (GRID_WIDTH * GRID_HEIGHT) // 가능한 최대 목표 지점(주차 공간) 수
//#define INF DBL_MAX             // 무한대 값을 double의 최대값으로 정의 (비용 계산에 사용)
//#define NUM_DIRECTIONS 4        // 에이전트가 이동할 수 있는 방향의 수 (상, 하, 좌, 우)
//
//// --- 시뮬레이션 동작 상수 ---
//#define DISTANCE_BEFORE_CHARGE 300.0 // 충전이 필요해지는 기준 이동 거리
//#define CHARGE_TIME 20               // 충전에 걸리는 시간 (시뮬레이션 스텝 기준)
//#define MAX_CHARGE_STATIONS 10       // 최대 충전소 개수
//#define MAX_PHASES 20                // 사용자 정의 시나리오에서 설정 가능한 최대 단계 수
//#define REALTIME_MODE_TIMELIMIT 10000 // 실시간 모드의 최대 시뮬레이션 시간
//#define MAX_TASKS 50                 // 작업 큐에 저장할 수 있는 최대 작업 수
//#define MAX_SPEED_MULTIPLIER 100.0f  // 시뮬레이션 최대 배속
//#define EVENT_GENERATION_INTERVAL 5 // 실시간 모드에서 이벤트(주차/출차 요청)가 발생하는 시간 간격
//
//// --- 로깅 및 UI 상수 ---
//#define LOG_BUFFER_LINES 5      // 화면에 표시할 로그 메시지의 최대 줄 수
//#define LOG_BUFFER_WIDTH 256    // 각 로그 메시지의 최대 길이
//#define STATUS_STRING_WIDTH 25  // 에이전트 상태 문자열을 출력할 때의 고정 너비 (UI 정렬용)
//
//// =============================================================================
//// --- 2. 구조체 정의 ---
//// =============================================================================
//// --- 기본 구조체 ---
///** @struct Key
// * @brief 우선순위 큐(priority queue)에서 노드의 우선순위를 결정하는 키 값.
// * LPA* 알고리즘에서 사용되며, k1과 k2 두 값으로 우선순위를 비교합니다.
// */
//typedef struct { double k1; double k2; } Key;
//
///** @struct Node
// * @brief 그리드 맵의 각 셀(노드)을 나타내는 구조체.
// */
//typedef struct Node {
//    int x, y;                   // 노드의 그리드 상 x, y 좌표
//    double g, rhs;              // LPA* 알고리즘에서 사용하는 g값(실제 비용)과 rhs값(예상 비용)
//    Key key;                    // 우선순위 큐에서의 정렬을 위한 키 값
//    int is_obstacle;            // 이 노드가 장애물인지 여부 (TRUE/FALSE)
//    int is_goal;                // 이 노드가 주차 공간(목표)인지 여부
//    int is_temp;                // 임시 장애물로 설정되었는지 여부 (다른 에이전트 회피용)
//    int is_parked;              // 이 노드에 차량이 주차되어 있는지 여부
//    int reserved_by_agent;      // 어떤 에이전트에 의해 예약되었는지 (에이전트 ID, -1은 예약 없음)
//    int in_pq;                  // 현재 우선순위 큐에 포함되어 있는지 여부
//    int pq_index;               // 우선순위 큐(힙) 내에서의 인덱스
//} Node;
//
///** @enum AgentState
// * @brief 에이전트의 현재 상태를 나타내는 열거형.
// */
//typedef enum {
//    IDLE,                       // 대기 상태
//    GOING_TO_PARK,              // 주차하러 가는 중
//    RETURNING_HOME_EMPTY,       // 주차 후 빈 몸으로 기지로 복귀하는 중
//    GOING_TO_COLLECT,           // 출차할 차를 가지러 가는 중
//    RETURNING_WITH_CAR,         // 차를 싣고 출차 지점으로 가는 중
//    GOING_TO_CHARGE,            // 충전소로 가는 중
//    CHARGING,                   // 충전 중
//    RETURNING_HOME_MAINTENANCE  // 충전 후 기지로 복귀하는 상태
//} AgentState;
//
///** @struct Agent
// * @brief 하나의 에이전트(로봇)를 나타내는 구조체.
// */
//typedef struct {
//    int id;                     // 에이전트의 고유 ID (0, 1, 2)
//    char symbol;                // 맵에 표시될 에이전트의 기호 ('A', 'B', 'C')
//    Node* pos;                  // 현재 위치를 가리키는 노드 포인터
//    Node* home_base;            // 에이전트의 기지(시작점)를 가리키는 노드
//    Node* goal;                 // 현재 목표 지점을 가리키는 노드 포인터
//    AgentState state;           // 에이전트의 현재 상태
//    double total_distance_traveled; // 총 이동 거리 (충전 필요 여부 판단에 사용)
//    int charge_timer;           // 충전 잔여 시간
//} Agent;
//
///** @struct PriorityQueue
// * @brief 최소 힙(min-heap)으로 구현된 우선순위 큐.
// * LPA* 알고리즘에서 비용이 가장 낮은 노드를 효율적으로 찾기 위해 사용됩니다.
// */
//typedef struct {
//    Node** nodes;   // 노드 포인터를 저장하는 배열
//    int size;       // 현재 큐에 저장된 요소의 개수
//    int capacity;   // 큐의 최대 용량
//} PriorityQueue;
//
///** @enum PhaseType
// * @brief 사용자 정의 시나리오의 각 단계 유형을 나타내는 열거형.
// */
//typedef enum {
//    PARK_PHASE, // 주차 단계
//    EXIT_PHASE  // 출차 단계
//} PhaseType;
//
///** @struct DynamicPhase
// * @brief 사용자 정의 시나리오의 한 단계를 정의하는 구조체.
// */
//typedef struct {
//    PhaseType type;             // 단계의 유형 (주차 또는 출차)
//    int task_count;             // 이 단계에서 수행해야 할 작업(차량)의 수
//    char type_name[10];         // 단계 유형을 문자열로 저장 (UI 표시용)
//} DynamicPhase;
//
///** @enum TaskType
// * @brief 실시간 모드에서 생성되는 작업의 유형을 나타내는 열거형.
// */
//typedef enum {
//    TASK_NONE,  // 작업 없음
//    TASK_PARK,  // 주차 작업
//    TASK_EXIT   // 출차 작업
//} TaskType;
//
///** @struct TaskNode
// * @brief 작업 큐의 단일 노드를 나타내는 연결 리스트 구조체.
// */
//typedef struct TaskNode {
//    TaskType type;
//    struct TaskNode* next;
//} TaskNode;
//
///** @enum SimulationMode
// * @brief 시뮬레이션의 전체 동작 모드를 나타내는 열거형.
// */
//typedef enum {
//    MODE_UNINITIALIZED, // 초기화되지 않은 상태
//    MODE_CUSTOM,        // 사용자 정의 시나리오 모드
//    MODE_REALTIME       // 실시간 모드
//} SimulationMode;
//
//// --- 모듈화된 구조체 ---
///** @struct GridMap
//* @brief 그리드, 주차 공간, 충전소 등 맵 관련 데이터를 관리.
//*/
//typedef struct {
//    Node grid[GRID_HEIGHT][GRID_WIDTH]; // 2D 그리드 맵
//    Node* goals[MAX_GOALS];             // 모든 주차 공간 노드를 가리키는 포인터 배열
//    int num_goals;                      // 총 주차 공간의 수
//    Node* charge_stations[MAX_CHARGE_STATIONS]; // 모든 충전소 노드를 가리키는 포인터 배열
//    int num_charge_stations;            // 총 충전소의 수
//} GridMap;
//
///** @struct AgentManager
//* @brief 모든 에이전트의 상태와 주차된 차량 수를 관리.
//*/
//typedef struct {
//    Agent agents[MAX_AGENTS];   // 에이전트 객체 배열
//    int total_cars_parked;      // 현재 주차장에 주차된 총 차량 수
//} AgentManager;
//
///** @struct Pathfinder
//* @brief LPA* 경로 탐색 중의 상태(PQ, 시작/목표)를 임시로 관리.
//*/
//typedef struct {
//    PriorityQueue pq;           // 경로 탐색을 위한 우선순위 큐
//    Node* start_node;           // 탐색 시작 노드
//    Node* goal_node;            // 탐색 목표 노드
//} Pathfinder;
//
///** @struct ScenarioManager
//* @brief 시뮬레이션 모드, 시나리오 단계, 작업 큐 등을 관리.
//*/
//typedef struct {
//    SimulationMode mode;                // 현재 시뮬레이션 모드
//    int time_step;                      // 시뮬레이션 경과 시간 (스텝)
//    int simulation_speed;               // 시뮬레이션 지연 시간 (ms)
//    float speed_multiplier;             // 시뮬레이션 배속
//    DynamicPhase phases[MAX_PHASES];    // 사용자 정의 시나리오의 단계들
//    int num_phases;                     // 총 단계 수
//    int current_phase_index;            // 현재 진행 중인 단계의 인덱스
//    int tasks_completed_in_phase;       // 현재 단계에서 완료된 작업 수
//    TaskNode* task_queue_head;          // 작업 큐(연결 리스트)의 시작
//    TaskNode* task_queue_tail;          // 작업 큐(연결 리스트)의 끝
//    int task_count;                     // 큐에 있는 작업의 수
//    int park_chance;                    // 실시간 모드에서 주차 요청이 발생할 확률
//    int exit_chance;                    // 실시간 모드에서 출차 요청이 발생할 확률
//} ScenarioManager;
//
///** @struct Logger
//* @brief 순환 로그 버퍼를 관리.
//*/
//typedef struct {
//    char logs[LOG_BUFFER_LINES][LOG_BUFFER_WIDTH]; // 로그 메시지를 저장하는 2D 배열
//    int log_head;                       // 가장 오래된 로그의 인덱스 (순환 버퍼용)
//    int log_count;                      // 현재 저장된 로그의 수
//} Logger;
//
///** @struct Simulation
//* @brief 모든 하위 모듈을 소유하고 전체 시뮬레이션을 오케스트레이션.
//*/
//typedef struct {
//    GridMap* map;                   // 맵 데이터 관리 모듈
//    AgentManager* agent_manager;    // 에이전트 관리 모듈
//    ScenarioManager* scenario_manager; // 시나리오 관리 모듈
//    Logger* logger;                 // 로그 관리 모듈
//} Simulation;
//
//// =============================================================================
//// --- 3. 함수 프로토타입 선언 ---
//// =============================================================================
//// --- 모듈 생성/소멸 함수 ---
//Simulation* simulation_create(); // Simulation 객체를 생성하고 초기화
//void simulation_destroy(Simulation* sim); // Simulation 객체와 하위 모듈들의 메모리를 해제
//
//// --- 시스템, UI 및 로깅 함수 ---
//void system_enable_virtual_terminal(); // Windows에서 ANSI 컬러 코드를 사용 가능하게 설정
//void ui_clear_screen_optimized(); // 화면을 지우는 최적화된 함수
//int simulation_setup(Simulation* sim); // 시뮬레이션 시작 전 모드, 시나리오 등을 설정
//void logger_log(Logger* logger, const char* format, ...); // 로그 버퍼에 메시지 추가
//
//// --- 우선순위 큐 (Priority Queue) 함수 ---
//void pq_init(PriorityQueue* pq, int capacity); // 우선순위 큐 초기화
//void pq_free(PriorityQueue* pq); // 우선순위 큐 메모리 해제
//void pq_push(PriorityQueue* pq, Node* node); // 큐에 노드 추가
//Node* pq_pop(PriorityQueue* pq); // 큐에서 우선순위가 가장 높은 노드 제거 및 반환
//void pq_remove(PriorityQueue* pq, Node* node); // 큐에서 특정 노드 제거
//int pq_contains(const Node* node); // 노드가 큐에 포함되어 있는지 확인
//Key pq_top_key(const PriorityQueue* pq); // 큐의 최상위 노드의 키 값을 확인 (제거하지 않음)
//
//// --- 그리드 (GridMap) 관리 함수 ---
//int grid_is_valid_coord(int x, int y); // 주어진 좌표가 그리드 범위 내에 있는지 확인
//int grid_is_node_blocked(const GridMap* map, const AgentManager* agent_manager, const Node* n); // 노드가 이동 불가능한지(장애물, 주차된 차 등) 확인
//
//// --- 경로 탐색 (Pathfinder) 함수 ---
//Pathfinder* pathfinder_create(Node* start, Node* goal); // 경로 탐색기 생성
//void pathfinder_destroy(Pathfinder* pf); // 경로 탐색기 메모리 해제
//void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, AgentManager* agent_manager); // LPA* 알고리즘으로 최단 경로 계산
//Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager, Node* current_node); // 계산된 경로를 따라 다음 이동할 노드 반환
//
//// --- 에이전트 (AgentManager) 로직 함수 ---
//void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]); // 모든 에이전트의 다음 움직임을 계획하고 충돌 해결
//void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger); // 이동 후 에이전트 상태 업데이트 (목표 도달 등)
//void agent_manager_update_charge_state(AgentManager* manager, Logger* logger); // 충전 중인 에이전트 상태 업데이트
//
//// --- 시뮬레이션 (Simulation) 관리 함수 ---
//void simulation_run(Simulation* sim); // 메인 시뮬레이션 루프 실행
//
//// =============================================================================
//// --- 4. 시스템, UI 및 로깅 구현 ---
//// =============================================================================
///**
// * @brief Windows 콘솔에서 ANSI 이스케이프 코드(컬러 등)를 사용 가능하게 합니다.
// */
//void system_enable_virtual_terminal() {
//#ifdef _WIN32 // 이 코드는 Windows에서만 컴파일됩니다.
//    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE); // 표준 출력 핸들을 가져옵니다.
//    if (hOut == INVALID_HANDLE_VALUE) return; // 핸들 가져오기 실패 시 함수 종료
//    DWORD dwMode = 0; // 콘솔 모드를 저장할 변수
//    if (!GetConsoleMode(hOut, &dwMode)) return; // 현재 콘솔 모드를 가져옵니다. 실패 시 종료
//    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING; // 가상 터미널 처리 모드를 활성화합니다.
//    SetConsoleMode(hOut, dwMode); // 변경된 콘솔 모드를 적용합니다.
//#endif
//}
//
///**
// * @brief 화면을 지웁니다. Windows에서는 커서 위치를 맨 위로 옮겨 깜빡임을 줄입니다.
// */
//void ui_clear_screen_optimized() {
//#ifdef _WIN32 // Windows 환경일 경우
//    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); // 표준 출력 핸들을 가져옵니다.
//    COORD coordScreen = { 0, 0 }; // 화면의 맨 위 왼쪽 좌표(0, 0)
//    SetConsoleCursorPosition(hConsole, coordScreen); // 콘솔 커서를 해당 좌표로 이동시킵니다.
//#else // Linux, macOS 등 다른 환경일 경우
//    printf("\033[H"); // ANSI 이스케이프 코드를 사용하여 커서를 홈 위치로 이동시킵니다.
//#endif
//}
//
///**
//* @brief Enter 키 없이 단일 문자를 즉시 입력받습니다. (Windows/Linux 호환)
//* @return 입력된 문자
//*/
//static char get_single_char() {
//#ifdef _WIN32 // Windows 환경
//    return _getch(); // _getch() 함수 사용
//#else // Linux/macOS 환경
//    char buf = 0; // 문자 버퍼
//    struct termios old = { 0 }; // 터미널 설정을 저장할 구조체
//    fflush(stdout); // 출력 버퍼 비우기
//    if (tcgetattr(0, &old) < 0) // 현재 터미널 설정 가져오기
//        perror("tcsetattr()");
//    old.c_lflag &= ~ICANON; // 정규 입력 모드(line-buffered) 비활성화
//    old.c_lflag &= ~ECHO;   // 입력 문자 화면에 표시(echo) 비활성화
//    old.c_cc[VMIN] = 1;     // 최소 1개의 문자를 읽을 때까지 대기
//    old.c_cc[VTIME] = 0;    // 타임아웃 없음
//    if (tcsetattr(0, TCSANOW, &old) < 0) // 변경된 터미널 설정 즉시 적용
//        perror("tcsetattr icanon");
//    if (read(0, &buf, 1) < 0) // 표준 입력에서 1바이트 읽기
//        perror("read()");
//    old.c_lflag |= ICANON;  // 터미널 설정을 원래대로 복원 (정규 모드 활성화)
//    old.c_lflag |= ECHO;    // 에코 활성화
//    if (tcsetattr(0, TCSADRAIN, &old) < 0) // 출력 완료 후 터미널 설정 복원
//        perror("tcsetattr ~icanon");
//    return buf; // 읽은 문자 반환
//#endif
//}
//
///**
//* @brief 사용자로부터 유효한 문자 중 하나를 입력받습니다.
//* @param prompt 사용자에게 보여줄 안내 메시지
//* @param valid_chars 유효한 문자들로 이루어진 문자열 (예: "abq")
//* @return 사용자가 입력한 유효한 문자
//*/
//static char get_char_input(const char* prompt, const char* valid_chars) {
//    char choice;
//    while (TRUE) { // 유효한 입력이 들어올 때까지 반복
//        printf("%s", prompt); // 안내 메시지 출력
//        choice = tolower(get_single_char()); // 단일 문자를 입력받아 소문자로 변환
//        printf("%c\n", choice); // 사용자에게 입력 피드백 제공
//        if (strchr(valid_chars, choice)) { // 입력된 문자가 유효한 문자 목록에 있는지 확인
//            return choice; // 유효하면 문자 반환
//        }
//        // 유효하지 않으면 오류 메시지 출력
//        printf(C_B_RED "\n잘못된 입력입니다. 유효한 키를 눌러주세요. (%s)\n" C_NRM, valid_chars);
//    }
//}
//
///**
// * @brief 사용자로부터 특정 범위 내의 정수를 입력받습니다.
// */
//static int get_integer_input(const char* prompt, int min, int max) {
//    char buffer[INPUT_BUFFER_SIZE]; // 입력 버퍼
//    int value;
//    while (TRUE) { // 유효한 입력이 들어올 때까지 반복
//        printf("%s", prompt); // 안내 메시지 출력
//        if (fgets(buffer, sizeof(buffer), stdin) != NULL) { // 한 줄을 입력받음
//            // 입력받은 문자열을 정수로 변환하고, 범위 내에 있는지 확인
//            if (sscanf(buffer, "%d", &value) == 1 && value >= min && value <= max) return value;
//        }
//        // 유효하지 않으면 오류 메시지 출력
//        printf(C_B_RED "잘못된 입력입니다. %d에서 %d 사이의 정수를 입력하세요.\n" C_NRM, min, max);
//    }
//}
//
///**
// * @brief 사용자로부터 특정 범위 내의 실수를 입력받습니다.
// */
//static float get_float_input(const char* prompt, float min, float max) {
//    char buffer[INPUT_BUFFER_SIZE]; // 입력 버퍼
//    float value;
//    while (TRUE) { // 유효한 입력이 들어올 때까지 반복
//        printf("%s", prompt); // 안내 메시지 출력
//        if (fgets(buffer, sizeof(buffer), stdin) != NULL) { // 한 줄을 입력받음
//            // 입력받은 문자열을 실수로 변환하고, 범위 내에 있는지 확인
//            if (sscanf(buffer, "%f", &value) == 1 && value >= min && value <= max) return value;
//        }
//        // 유효하지 않으면 오류 메시지 출력
//        printf(C_B_RED "잘못된 입력입니다. %.1f에서 %.1f 사이의 숫자를 입력하세요.\n" C_NRM, min, max);
//    }
//}
//
///**
// * @brief 사용자 정의 시나리오를 설정합니다.
// */
//static int simulation_setup_custom_scenario(ScenarioManager* scenario) {
//    printf(C_B_WHT "--- 사용자 정의 시나리오 설정 ---\n" C_NRM);
//    scenario->num_phases = get_integer_input(C_YEL "총 단계 수를 입력하세요 (1-20, 0=취소): " C_NRM, 0, MAX_PHASES);
//    if (scenario->num_phases == 0) return 0; // 0을 입력하면 취소
//
//    for (int i = 0; i < scenario->num_phases; i++) { // 각 단계에 대해 설정
//        printf(C_B_CYN "\n--- %d/%d 단계 설정 ---\n" C_NRM, i + 1, scenario->num_phases);
//        printf("a. %s주차%s\n", C_YEL, C_NRM);
//        printf("b. %s출차%s\n", C_CYN, C_NRM);
//        char type_choice = get_char_input("단계 유형을 선택하세요: ", "ab");
//        scenario->phases[i].task_count = get_integer_input("이 단계에서 처리할 차량 수를 입력하세요: ", 1, 100);
//
//        if (type_choice == 'a') { // 'a'를 선택하면 주차 단계
//            scenario->phases[i].type = PARK_PHASE;
//            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "주차");
//        }
//        else { // 'b'를 선택하면 출차 단계
//            scenario->phases[i].type = EXIT_PHASE;
//            snprintf(scenario->phases[i].type_name, sizeof(scenario->phases[i].type_name), "출차");
//        }
//        printf(C_GRN "%d단계 설정 완료: %s %d대.\n" C_NRM, i + 1, scenario->phases[i].type_name, scenario->phases[i].task_count);
//    }
//    printf(C_B_GRN "\n--- 시나리오 설정이 완료되었습니다! ---\n" C_NRM);
//    sleep(1500); // 1.5초 대기
//    return 1; // 성공
//}
//
///**
// * @brief 실시간 시뮬레이션의 파라미터를 설정합니다.
// */
//static int simulation_setup_realtime(ScenarioManager* scenario) {
//    printf(C_B_WHT "--- 실시간 시뮬레이션 설정 ---\n" C_NRM);
//    while (TRUE) { // 유효한 확률값이 입력될 때까지 반복
//        scenario->park_chance = get_integer_input("\n주차 요청 발생 확률(0~100)을 입력하세요: ", 0, 100);
//        scenario->exit_chance = get_integer_input("출차 요청 발생 확률(0~100)을 입력하세요: ", 0, 100);
//        if ((scenario->park_chance + scenario->exit_chance) <= 100) break; // 두 확률의 합이 100 이하면 루프 탈출
//        printf(C_B_RED "주차와 출차 확률의 합은 100을 넘을 수 없습니다.\n" C_NRM);
//    }
//    printf(C_B_GRN "\n설정 완료: 주차 확률 %d%%, 출차 확률 %d%%\n" C_NRM, scenario->park_chance, scenario->exit_chance);
//    sleep(1500); // 1.5초 대기
//    return 1; // 성공
//}
//
///**
// * @brief 시뮬레이션 속도를 설정합니다.
// */
//static int simulation_setup_speed(ScenarioManager* scenario) {
//    printf(C_B_WHT "\n--- 시뮬레이션 속도 설정 ---\n" C_NRM);
//    scenario->speed_multiplier = get_float_input("원하는 배속을 입력하세요 (1.0 ~ 100.0): ", 1.0f, MAX_SPEED_MULTIPLIER);
//    // 배속에 반비례하여 지연 시간 설정
//    scenario->simulation_speed = (int)(100.0f / scenario->speed_multiplier);
//    if (scenario->simulation_speed < 1) scenario->simulation_speed = 1; // 최소 지연 시간은 1ms
//    printf(C_B_GRN "\n--- 시뮬레이션을 %.1fx 배속으로 시작합니다... ---\n" C_NRM, scenario->speed_multiplier);
//    sleep(1500); // 1.5초 대기
//    return 1; // 성공
//}
//
///**
// * @brief 시뮬레이션 시작 전 전체 설정을 담당합니다.
// */
//int simulation_setup(Simulation* sim) {
//    ui_clear_screen_optimized(); // 화면 지우기
//    printf(C_B_WHT "--- 시뮬레이션 모드 선택 ---\n" C_NRM);
//    printf("a. %s사용자 정의 시나리오%s\n", C_YEL, C_NRM);
//    printf("b. %s실시간 시뮬레이션%s\n", C_CYN, C_NRM);
//    printf("q. %s종료%s\n\n", C_RED, C_NRM);
//
//    char choice = get_char_input("실행할 시나리오 문자를 입력하세요: ", "abq"); // 사용자로부터 모드 선택
//    int setup_success = 0;
//    switch (choice) {
//    case 'a': // 사용자 정의 시나리오
//        sim->scenario_manager->mode = MODE_CUSTOM;
//        if (simulation_setup_custom_scenario(sim->scenario_manager)) { // 시나리오 설정이 성공하면
//            setup_success = simulation_setup_speed(sim->scenario_manager); // 속도 설정
//        }
//        break;
//    case 'b': // 실시간 시뮬레이션
//        sim->scenario_manager->mode = MODE_REALTIME;
//        if (simulation_setup_realtime(sim->scenario_manager)) { // 실시간 설정이 성공하면
//            setup_success = simulation_setup_speed(sim->scenario_manager); // 속도 설정
//        }
//        break;
//    case 'q': // 종료
//        return 0; // 실패(종료) 반환
//    }
//
//    if (setup_success) { // 설정이 성공적으로 완료되면
//        ui_clear_screen_optimized(); // 시뮬레이션 시작 전 화면을 깨끗하게 정리
//    }
//    return setup_success; // 설정 성공 여부 반환
//}
//
///**
//* @brief 그리드 맵의 현재 상태를 문자열 버퍼에 렌더링합니다.
//* @param buffer 렌더링된 문자열을 저장할 버퍼
//* @param buffer_size 버퍼의 전체 크기
//* @param map GridMap 객체
//* @param agent_manager AgentManager 객체
//* @return 버퍼에 쓰여진 바이트 수
//*/
//static int grid_map_render_to_buffer(char* buffer, size_t buffer_size, const GridMap* map, const AgentManager* agent_manager) {
//    char view[GRID_HEIGHT][GRID_WIDTH]; // 화면에 표시될 문자를 저장할 2D 배열
//    const char* colors[GRID_HEIGHT][GRID_WIDTH]; // 각 문자의 색상을 저장할 2D 배열
//    char* buf_ptr = buffer; // 버퍼의 현재 위치를 가리키는 포인터
//    size_t remaining_size = buffer_size; // 버퍼의 남은 크기
//    int written = 0; // 한 번에 쓰여진 바이트 수
//
//    // 1. 뷰와 색상 매트릭스 초기화 (기본 타일)
//    for (int y = 0; y < GRID_HEIGHT; y++) {
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            if (map->grid[y][x].is_obstacle) { // 장애물인 경우
//                view[y][x] = '+'; colors[y][x] = C_WHT;
//            }
//            else { // 길인 경우
//                view[y][x] = '.'; colors[y][x] = C_GRY;
//            }
//        }
//    }
//
//    // 2. 특수 노드들(충전소, 주차 공간) 그리기
//    for (int i = 0; i < map->num_charge_stations; i++) { // 모든 충전소에 대해
//        Node* cs = map->charge_stations[i];
//        view[cs->y][cs->x] = 'e'; // 'e'로 표시
//        int is_charging_here = FALSE; // 해당 위치에서 충전 중인지 확인하는 플래그
//        for (int j = 0; j < MAX_AGENTS; j++) {
//            if (agent_manager->agents[j].state == CHARGING && agent_manager->agents[j].pos == cs) {
//                is_charging_here = TRUE; break;
//            }
//        }
//        colors[cs->y][cs->x] = is_charging_here ? C_B_RED : C_B_YEL; // 충전 중이면 빨간색, 아니면 노란색
//    }
//    for (int i = 0; i < map->num_goals; i++) { // 모든 주차 공간에 대해
//        Node* g = map->goals[i];
//        if (g->is_parked) { // 주차된 경우
//            view[g->y][g->x] = 'P'; colors[g->y][g->x] = C_RED;
//        }
//        else if (g->is_goal) { // 비어있는 주차 공간인 경우
//            view[g->y][g->x] = 'G'; colors[g->y][g->x] = C_GRN;
//        }
//    }
//
//    // 3. 에이전트 그리기 (다른 요소들을 덮어쓰도록 마지막에 그림)
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        if (agent_manager->agents[i].pos) { // 에이전트 위치가 유효하면
//            view[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = agent_manager->agents[i].symbol;
//            if (i == 0) colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_CYN;
//            else if (i == 1) colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_YEL;
//            else colors[agent_manager->agents[i].pos->y][agent_manager->agents[i].pos->x] = C_B_MAG;
//        }
//    }
//
//    // 4. 완성된 뷰와 색상 매트릭스를 기반으로 버퍼에 문자열 쓰기
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
//    return buf_ptr - buffer; // 총 쓰여진 바이트 수 반환
//}
//
///**
//* @brief 시뮬레이션의 모든 상태 정보를 버퍼에 쓴 뒤, 한 번에 화면에 출력합니다.
//* @param sim Simulation 객체
//*/
//static void simulation_display_status(const Simulation* sim) {
//    const ScenarioManager* scenario = sim->scenario_manager;
//    const AgentManager* agent_manager = sim->agent_manager;
//    const GridMap* map = sim->map;
//    const Logger* logger = sim->logger;
//    char display_buffer[DISPLAY_BUFFER_SIZE]; // 화면 전체를 담을 큰 버퍼
//    char* buf_ptr = display_buffer; // 버퍼 포인터
//    size_t remaining_size = sizeof(display_buffer); // 남은 버퍼 크기
//    int written = 0; // 쓰여진 바이트 수
//
//    // --- 1. 상태 헤더 정보 버퍼에 쓰기 ---
//    written = snprintf(buf_ptr, remaining_size, C_B_WHT);
//    buf_ptr += written; remaining_size -= written;
//
//    if (scenario->mode == MODE_CUSTOM) { // 사용자 정의 모드일 때
//        if (scenario->current_phase_index < scenario->num_phases) {
//            DynamicPhase* p = &scenario->phases[scenario->current_phase_index];
//            written = snprintf(buf_ptr, remaining_size, "--- Custom Scenario: %d/%d [Speed: %.1fx] ---\n", scenario->current_phase_index + 1, scenario->num_phases, scenario->speed_multiplier);
//            buf_ptr += written; remaining_size -= written;
//            written = snprintf(buf_ptr, remaining_size, "Time: %d, Current Task: %s (%d/%d)\n", scenario->time_step, p->type_name, scenario->tasks_completed_in_phase, p->task_count);
//            buf_ptr += written; remaining_size -= written;
//        }
//        else { // 모든 단계 완료
//            written = snprintf(buf_ptr, remaining_size, "--- Custom Scenario: All phases complete ---\n");
//            buf_ptr += written; remaining_size -= written;
//        }
//    }
//    else if (scenario->mode == MODE_REALTIME) { // 실시간 모드일 때
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
//            C_B_GRN, park_requests, C_NRM, // 주차 요청은 밝은 초록색
//            C_B_YEL, exit_requests, C_NRM  // 출차 요청은 밝은 노란색
//        );
//        buf_ptr += written; remaining_size -= written;
//    }
//    written = snprintf(buf_ptr, remaining_size, "Parked Cars: %d/%d\n" C_NRM, agent_manager->total_cars_parked, map->num_goals);
//    buf_ptr += written; remaining_size -= written;
//
//    // --- 2. 그리드 맵 정보 버퍼에 쓰기 ---
//    written = grid_map_render_to_buffer(buf_ptr, remaining_size, map, agent_manager);
//    buf_ptr += written; remaining_size -= written;
//
//    // --- 3. 에이전트 상태 정보 버퍼에 쓰기 ---
//    const char* agent_state_strings[] = { "대기", "주차 중", "기지 복귀(빈 차)", "수거 중", "출차 중(차량 탑승)", "충전소로 이동", "충전 중", "기지 복귀(충전 후)" };
//    const char* agent_state_colors[] = { C_GRY, C_YEL, C_CYN, C_YEL, C_GRN, C_B_RED, C_RED, C_CYN };
//
//
//    for (int i = 0; i < MAX_AGENTS; i++) { // 모든 에이전트에 대해
//        const Agent* agent = &agent_manager->agents[i];
//        const char* agent_color = (i == 0) ? C_B_CYN : (i == 1) ? C_B_YEL : C_B_MAG;
//        char status_buffer[100]; // 상태 문자열을 임시로 저장할 버퍼
//
//        if (agent->state == CHARGING) { // 충전 중일 경우 타이머 표시
//            snprintf(status_buffer, sizeof(status_buffer), "충전 중... (%d)", agent->charge_timer);
//        }
//        else { // 그 외 상태
//            snprintf(status_buffer, sizeof(status_buffer), "%s", agent_state_strings[agent->state]);
//        }
//
//        // 에이전트 ID와 현재 위치 (x,y)
//        written = snprintf(buf_ptr, remaining_size, "%sAgent %c%s: (%2d,%d) ", agent_color, agent->symbol, C_NRM, agent->pos->x, agent->pos->y);
//        buf_ptr += written; remaining_size -= written;
//
//        if (agent->goal) { // 목표가 있을 경우 목표 위치
//            written = snprintf(buf_ptr, remaining_size, "-> (%2d,%d) ", agent->goal->x, agent->goal->y);
//        }
//        else { // 목표가 없을 경우
//            written = snprintf(buf_ptr, remaining_size, "-> 없음       ");
//        }
//        buf_ptr += written; remaining_size -= written;
//
//        // 이동 거리(마일리지)와 상태 문자열
//        written = snprintf(buf_ptr, remaining_size, "[Mileage: %6.1f/%d] [%s%-*s%s]\n",
//            agent->total_distance_traveled, (int)DISTANCE_BEFORE_CHARGE,
//            agent_state_colors[agent->state], STATUS_STRING_WIDTH, status_buffer, C_NRM);
//        buf_ptr += written; remaining_size -= written;
//    }
//    written = snprintf(buf_ptr, remaining_size, "\n");
//    buf_ptr += written; remaining_size -= written;
//
//    // --- 4. 로그 정보 버퍼에 쓰기 ---
//    written = snprintf(buf_ptr, remaining_size, C_B_WHT "--- Simulation Log ---\n" C_NRM);
//    buf_ptr += written; remaining_size -= written;
//    for (int i = 0; i < logger->log_count; i++) {
//        int index = (logger->log_head + i) % LOG_BUFFER_LINES; // 순환 버퍼 인덱스 계산
//        written = snprintf(buf_ptr, remaining_size, "%s%s%s\n", C_GRY, logger->logs[index], C_NRM);
//        buf_ptr += written; remaining_size -= written;
//    }
//
//    // --- 5. 완성된 버퍼를 화면에 한 번에 출력 ---
//    ui_clear_screen_optimized(); // 화면을 지우고
//    printf("%s", display_buffer); // 버퍼의 내용을 한 번에 출력 (깜빡임 최소화)
//}
//
//// --- Logger ---
///**
// * @brief Logger 객체를 생성하고 메모리를 할당합니다.
// */
//Logger* logger_create() {
//    Logger* logger = (Logger*)calloc(1, sizeof(Logger)); // Logger 구조체 크기만큼 메모리를 할당하고 0으로 초기화
//    if (!logger) { // 메모리 할당 실패 시
//        perror("Logger 할당 실패"); // 오류 메시지 출력
//        exit(1); // 프로그램 종료
//    }
//    return logger; // 생성된 Logger 객체 포인터 반환
//}
//
///**
// * @brief Logger 객체의 메모리를 해제합니다.
// */
//void logger_destroy(Logger* logger) {
//    if (logger) free(logger); // logger가 NULL이 아니면 메모리 해제
//}
//
///**
// * @brief 순환 로그 버퍼에 로그 메시지를 추가합니다.
// */
//void logger_log(Logger* logger, const char* format, ...) {
//    va_list args; // 가변 인자 목록
//    va_start(args, format); // 가변 인자 목록 초기화
//    // 로그를 저장할 다음 위치 계산 (순환 버퍼)
//    int current_log_index = (logger->log_head + logger->log_count) % LOG_BUFFER_LINES;
//    // 포맷에 맞춰 로그 메시지를 버퍼에 씀
//    vsnprintf(logger->logs[current_log_index], LOG_BUFFER_WIDTH, format, args);
//    va_end(args); // 가변 인자 목록 정리
//
//    if (logger->log_count < LOG_BUFFER_LINES) { // 버퍼가 아직 가득 차지 않았다면
//        logger->log_count++; // 로그 카운트 증가
//    }
//    else { // 버퍼가 가득 찼다면
//        logger->log_head = (logger->log_head + 1) % LOG_BUFFER_LINES; // 가장 오래된 로그를 덮어쓰도록 head를 이동
//    }
//}
//
//// =============================================================================
//// --- 5. 유틸리티 구현 (우선순위 큐) ---
//// =============================================================================
//// --- Priority Queue (min-heap) ---
///**
// * @brief 두 개의 Key 구조체를 비교합니다. k1을 먼저, 같으면 k2를 비교합니다.
// * @return k1 < k2 이면 -1, k1 > k2 이면 1, 같으면 0을 반환합니다.
// */
//static int compare_keys(Key k1, Key k2) {
//    if (k1.k1 < k2.k1 - 1e-9) return -1; // 부동소수점 오차를 고려하여 k1.k1이 작은지 비교
//    if (k1.k1 > k2.k1 + 1e-9) return 1;  // 부동소수점 오차를 고려하여 k1.k1이 큰지 비교
//    if (k1.k2 < k2.k2 - 1e-9) return -1; // k1.k1이 같을 경우, k1.k2가 작은지 비교
//    if (k1.k2 > k2.k2 + 1e-9) return 1;  // k1.k1이 같을 경우, k1.k2가 큰지 비교
//    return 0; // 두 키가 동일함
//}
//
///**
// * @brief 우선순위 큐를 초기화합니다.
// */
//void pq_init(PriorityQueue* pq, int capacity) {
//    pq->nodes = (Node**)malloc(sizeof(Node*) * capacity); // 노드 포인터 배열을 위한 메모리 할당
//    pq->size = 0; // 현재 크기를 0으로 설정
//    pq->capacity = capacity; // 최대 용량 설정
//}
//
///**
// * @brief 우선순위 큐의 메모리를 해제합니다.
// */
//void pq_free(PriorityQueue* pq) {
//    if (pq && pq->nodes) { // 큐와 노드 배열이 유효한 경우
//        free(pq->nodes); // 노드 포인터 배열 메모리 해제
//        pq->nodes = NULL; // 댕글링 포인터 방지
//    }
//}
//
///**
// * @brief 힙 내의 두 노드의 위치를 바꿉니다.
// */
//static void swap_nodes(Node** a, Node** b) {
//    Node* t = *a; *a = *b; *b = t; // 포인터를 이용한 스왑
//    int temp_idx = (*a)->pq_index;
//    (*a)->pq_index = (*b)->pq_index;
//    (*b)->pq_index = temp_idx; // 각 노드가 가진 pq_index 값도 스왑
//}
//
///**
// * @brief 힙의 특정 위치에서 위로 올라가며 힙 속성을 만족시키도록 재정렬합니다. (heapify up)
// */
//static void heapify_up(PriorityQueue* pq, int i) {
//    if (i == 0) return; // 루트 노드이면 종료
//    int p = (i - 1) / 2; // 부모 노드의 인덱스 계산
//    if (compare_keys(pq->nodes[i]->key, pq->nodes[p]->key) < 0) { // 자식 노드의 키 값이 부모보다 작으면
//        swap_nodes(&pq->nodes[i], &pq->nodes[p]); // 자식과 부모를 스왑
//        heapify_up(pq, p); // 부모 위치에서 다시 heapify_up 재귀 호출
//    }
//}
//
///**
// * @brief 힙의 특정 위치에서 아래로 내려가며 힙 속성을 만족시키도록 재정렬합니다. (heapify down)
// */
//static void heapify_down(PriorityQueue* pq, int i) {
//    int l = 2 * i + 1, r = 2 * i + 2, s = i; // l: 왼쪽 자식, r: 오른쪽 자식, s: 가장 작은 값을 가진 노드의 인덱스
//    if (l < pq->size && compare_keys(pq->nodes[l]->key, pq->nodes[s]->key) < 0) s = l; // 왼쪽 자식이 더 작으면 s를 l로
//    if (r < pq->size && compare_keys(pq->nodes[r]->key, pq->nodes[s]->key) < 0) s = r; // 오른쪽 자식이 더 작으면 s를 r로
//    if (s != i) { // 현재 노드(i)가 가장 작은 값이 아니면
//        swap_nodes(&pq->nodes[i], &pq->nodes[s]); // 가장 작은 값을 가진 자식과 스왑
//        heapify_down(pq, s); // 스왑된 위치에서 다시 heapify_down 재귀 호출
//    }
//}
//
///**
// * @brief 우선순위 큐에 노드를 추가합니다.
// */
//void pq_push(PriorityQueue* pq, Node* n) {
//    if (pq->size >= pq->capacity) return; // 큐가 가득 찼으면 종료
//    n->in_pq = TRUE; // 노드가 큐에 있음을 표시
//    n->pq_index = pq->size; // 노드의 힙 인덱스를 현재 큐의 마지막 위치로 설정
//    pq->nodes[pq->size++] = n; // 배열의 마지막에 노드를 추가하고 사이즈 증가
//    heapify_up(pq, pq->size - 1); // 추가된 위치에서 heapify_up 수행
//}
//
///**
// * @brief 우선순위 큐에서 가장 우선순위가 높은(키 값이 작은) 노드를 제거하고 반환합니다.
// */
//Node* pq_pop(PriorityQueue* pq) {
//    if (pq->size == 0) return NULL; // 큐가 비어있으면 NULL 반환
//    Node* top = pq->nodes[0]; // 루트 노드(가장 작은 값)를 top에 저장
//    top->in_pq = FALSE; // 큐에서 제거되었음을 표시
//    top->pq_index = -1; // 큐 인덱스를 -1로 리셋
//    pq->size--; // 큐 사이즈 감소
//    if (pq->size > 0) { // 큐에 노드가 남아있으면
//        pq->nodes[0] = pq->nodes[pq->size]; // 가장 마지막 노드를 루트로 이동
//        pq->nodes[0]->pq_index = 0; // 루트 노드의 인덱스를 0으로 설정
//        heapify_down(pq, 0); // 루트에서부터 heapify_down 수행
//    }
//    return top; // 저장해둔 top 노드 반환
//}
//
///**
// * @brief 우선순위 큐에서 특정 노드를 제거합니다.
// */
//void pq_remove(PriorityQueue* pq, Node* n) {
//    if (!n->in_pq) return; // 노드가 큐에 없으면 종료
//    int idx = n->pq_index; // 제거할 노드의 인덱스
//    pq->size--; // 큐 사이즈 감소
//    if (idx != pq->size) { // 제거할 노드가 마지막 노드가 아니었다면
//        pq->nodes[idx] = pq->nodes[pq->size]; // 마지막 노드를 제거할 노드의 위치로 이동
//        pq->nodes[idx]->pq_index = idx; // 인덱스 업데이트
//        // 이동한 노드의 키 값에 따라 heapify_up 또는 heapify_down 수행
//        if (idx > 0 && compare_keys(pq->nodes[idx]->key, pq->nodes[(idx - 1) / 2]->key) < 0) {
//            heapify_up(pq, idx);
//        }
//        else {
//            heapify_down(pq, idx);
//        }
//    }
//    n->in_pq = FALSE; // 큐에서 제거되었음을 표시
//    n->pq_index = -1; // 큐 인덱스 리셋
//}
//
///**
// * @brief 노드가 우선순위 큐에 포함되어 있는지 확인합니다.
// */
//int pq_contains(const Node* n) { return n->in_pq; } // 노드의 in_pq 플래그 반환
//
///**
// * @brief 우선순위 큐의 최상위(가장 작은) 키 값을 반환합니다. (노드를 제거하지 않음)
// */
//Key pq_top_key(const PriorityQueue* pq) {
//    if (pq->size == 0) return (Key) { INF, INF }; // 큐가 비어있으면 무한대 키 값 반환
//    return pq->nodes[0]->key; // 루트 노드의 키 값 반환
//}
//
//
//// =============================================================================
//// --- 6. 맵 관리 구현 (GridMap) ---
//// =============================================================================
///**
// * @brief 코드 내에 하드코딩된 맵 데이터로부터 맵을 로드합니다.
// */
//static void grid_map_load_from_embedded_map(GridMap* map, AgentManager* agent_manager) {
//    // 2D 주차장 맵 데이터
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
//    map->num_goals = 0; // 목표(주차 공간) 수 초기화
//    map->num_charge_stations = 0; // 충전소 수 초기화
//    int map_idx = 0; // 맵 데이터 문자열을 읽을 인덱스
//    for (int y = 0; y < GRID_HEIGHT; y++) { // 맵의 높이만큼 반복
//        for (int x = 0; x < GRID_WIDTH; x++) { // 맵의 너비만큼 반복
//            char ch;
//            do { ch = embedded_map_data[map_idx++]; } while (ch == '\n' || ch == '\r'); // 개행 문자는 건너뜀
//            if (ch == '\0') { // 맵 데이터가 예상보다 짧으면 오류 처리
//                fprintf(stderr, C_B_RED "오류: 내장된 맵 데이터가 예상보다 짧습니다.\n" C_NRM);
//                exit(1);
//            }
//
//            Node* n = &map->grid[y][x]; // 현재 (x, y) 좌표의 노드에 대한 포인터
//            // 노드 기본값으로 초기화
//            *n = (Node){ .x = x, .y = y, .g = INF, .rhs = INF, .is_obstacle = FALSE,
//            .is_goal = FALSE, .is_temp = FALSE, .is_parked = FALSE,
//            .reserved_by_agent = -1, .in_pq = FALSE, .pq_index = -1 };
//
//            switch (ch) { // 읽어온 문자에 따라 노드 속성 설정
//            case '1': n->is_obstacle = TRUE; break; // '1'은 장애물
//            case 'A': agent_manager->agents[0].pos = n; agent_manager->agents[0].home_base = n; break; // 'A'는 에이전트 0의 시작 위치
//            case 'B': agent_manager->agents[1].pos = n; agent_manager->agents[1].home_base = n; break; // 'B'는 에이전트 1의 시작 위치
//            case 'C': agent_manager->agents[2].pos = n; agent_manager->agents[2].home_base = n; break; // 'C'는 에이전트 2의 시작 위치
//            case 'G': n->is_goal = TRUE; if (map->num_goals < MAX_GOALS) map->goals[map->num_goals++] = n; break; // 'G'는 주차 공간 (목표)
//            case 'e': if (map->num_charge_stations < MAX_CHARGE_STATIONS) map->charge_stations[map->num_charge_stations++] = n; break; // 'e'는 충전소
//            }
//        }
//    }
//    if (map->num_charge_stations == 0) { // 맵에 충전소가 하나도 없으면 오류 처리
//        fprintf(stderr, C_B_RED "오류: 맵 데이터에 충전소('e')가 없습니다.\n" C_NRM);
//        exit(1);
//    }
//}
//
///**
// * @brief GridMap 객체를 생성하고 초기화합니다.
// */
//GridMap* grid_map_create(AgentManager* agent_manager) {
//    GridMap* map = (GridMap*)calloc(1, sizeof(GridMap)); // GridMap 구조체 메모리 할당 및 0으로 초기화
//    if (!map) { // 할당 실패 시
//        perror("GridMap 할당 실패");
//        exit(1);
//    }
//    grid_map_load_from_embedded_map(map, agent_manager); // 내장된 맵 데이터 로드
//    return map; // 생성된 map 객체 포인터 반환
//}
//
///**
// * @brief GridMap 객체의 메모리를 해제합니다.
// */
//void grid_map_destroy(GridMap* map) {
//    if (map) free(map); // map이 NULL이 아니면 메모리 해제
//}
//
///**
// * @brief 주어진 좌표가 그리드 범위 내에 있는지 확인합니다.
// */
//int grid_is_valid_coord(int x, int y) {
//    return x >= 0 && x < GRID_WIDTH&& y >= 0 && y < GRID_HEIGHT; // x, y가 0 이상이고 최대 너비/높이 미만인지 확인
//}
//
///**
// * @brief 주어진 노드가 현재 이동 불가능한 상태인지 확인합니다.
// */
//int grid_is_node_blocked(const GridMap* map, const AgentManager* agent_manager, const Node* n) {
//    if (n->is_obstacle || n->is_parked || n->is_temp) { // 고정 장애물, 주차된 차, 임시 장애물이면
//        return TRUE; // 막힘
//    }
//    for (int i = 0; i < MAX_AGENTS; i++) { // 모든 에이전트에 대해
//        // 다른 에이전트가 해당 위치에서 충전 중이라면
//        if (agent_manager->agents[i].pos == n && agent_manager->agents[i].state == CHARGING) {
//            return TRUE; // 막힘
//        }
//    }
//    return FALSE; // 그 외에는 막히지 않음
//}
//
//// =============================================================================
//// --- 7. 시나리오 및 작업 관리 구현 (ScenarioManager) ---
//// =============================================================================
///**
// * @brief ScenarioManager의 작업 큐를 비우고 모든 노드의 메모리를 해제합니다.
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
// * @brief ScenarioManager 객체를 생성하고 초기화합니다.
// */
//ScenarioManager* scenario_manager_create() {
//    ScenarioManager* manager = (ScenarioManager*)calloc(1, sizeof(ScenarioManager)); // 메모리 할당 및 0으로 초기화
//    if (!manager) { // 할당 실패 시
//        perror("ScenarioManager 할당 실패");
//        exit(1);
//    }
//    manager->simulation_speed = 100; // 기본 시뮬레이션 속도 (지연 시간)
//    manager->speed_multiplier = 1.0f; // 기본 배속
//    manager->park_chance = 40; // 기본 주차 요청 확률
//    manager->exit_chance = 30; // 기본 출차 요청 확률
//    // 연결 리스트 큐 초기화
//    manager->task_queue_head = NULL;
//    manager->task_queue_tail = NULL;
//    manager->task_count = 0;
//    return manager; // 생성된 manager 객체 포인터 반환
//}
//
///**
// * @brief ScenarioManager 객체의 메모리를 해제합니다.
// */
//void scenario_manager_destroy(ScenarioManager* manager) {
//    if (manager) {
//        scenario_manager_clear_task_queue(manager); // 큐에 남은 작업 메모리 해제
//        free(manager); // manager가 NULL이 아니면 메모리 해제
//    }
//}
//
///**
// * @brief 작업 큐(연결 리스트)에 새로운 작업을 추가합니다. (실시간 모드용)
// */
//static void add_task_to_queue(ScenarioManager* scenario, TaskType type) {
//    if (scenario->task_count >= MAX_TASKS) return; // 큐가 가득 찼으면 추가하지 않음
//
//    TaskNode* new_task = (TaskNode*)malloc(sizeof(TaskNode));
//    if (!new_task) {
//        perror("Failed to allocate memory for new task");
//        return;
//    }
//    new_task->type = type;
//    new_task->next = NULL;
//
//    if (scenario->task_queue_head == NULL) { // 큐가 비어있을 경우
//        scenario->task_queue_head = new_task;
//        scenario->task_queue_tail = new_task;
//    }
//    else { // 큐에 노드가 있을 경우
//        scenario->task_queue_tail->next = new_task;
//        scenario->task_queue_tail = new_task;
//    }
//    scenario->task_count++;
//}
//
//// =============================================================================
//// --- 8. 경로 탐색 구현 (Pathfinder) ---
//// =============================================================================
///**
// * @brief 두 노드 간의 맨해튼 거리를 계산하는 휴리스틱 함수.
// */
//static double heuristic(const Node* a, const Node* b) {
//    return fabs(a->x - b->x) + fabs(a->y - b->y); // |x1 - x2| + |y1 - y2|
//}
//
///**
// * @brief 두 노드 간의 유클리드 거리를 계산합니다.
// */
//static double euclidean_distance(const Node* a, const Node* b) {
//    return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2)); // sqrt((x1-x2)^2 + (y1-y2)^2)
//}
//
///**
// * @brief LPA* 알고리즘에 따라 노드의 우선순위 키를 계산합니다.
// */
//static Key calculate_key(const Pathfinder* pf, const Node* n) {
//    double m = fmin(n->g, n->rhs); // g값과 rhs값 중 작은 값을 선택
//    return (Key) { m + heuristic(n, pf->start_node), m }; // k1 = min(g, rhs) + h, k2 = min(g, rhs)
//}
//
///**
// * @brief 노드의 rhs 값을 업데이트하고, 필요 시 우선순위 큐에 추가/업데이트합니다.
// */
//static void path_update_vertex(Pathfinder* pf, GridMap* map, const AgentManager* agent_manager, Node* u) {
//    if (u != pf->goal_node) { // 목표 노드가 아니라면
//        double min_rhs = INF; // 최소 rhs 값을 무한대로 초기화
//        int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 }; // 상하좌우 이동
//        for (int i = 0; i < NUM_DIRECTIONS; i++) { // 4방향에 대해
//            int nx = u->x + dx[i], ny = u->y + dy[i]; // 이웃 노드 좌표 계산
//            if (grid_is_valid_coord(nx, ny)) { // 이웃이 그리드 내에 있다면
//                Node* successor = &map->grid[ny][nx]; // 이웃 노드 포인터
//                if (!grid_is_node_blocked(map, agent_manager, successor)) { // 이웃이 막혀있지 않다면
//                    min_rhs = fmin(min_rhs, successor->g + 1.0); // rhs 값을 갱신 (g(s') + c(s', s))
//                }
//            }
//        }
//        u->rhs = min_rhs; // 계산된 최소값으로 rhs 업데이트
//    }
//
//    if (pq_contains(u)) { // 노드가 이미 큐에 있다면
//        pq_remove(&pf->pq, u); // 일단 제거
//    }
//
//    // g와 rhs가 일치하지 않으면 (inconsistent)
//    if (fabs(u->g - u->rhs) > 1e-9) {
//        u->key = calculate_key(pf, u); // 키 값을 다시 계산하고
//        pq_push(&pf->pq, u); // 큐에 다시 추가 (우선순위가 갱신됨)
//    }
//}
//
///**
// * @brief 경로 탐색기(Pathfinder)를 생성합니다.
// */
//Pathfinder* pathfinder_create(Node* start, Node* goal) {
//    Pathfinder* pf = (Pathfinder*)malloc(sizeof(Pathfinder)); // 메모리 할당
//    if (!pf) return NULL; // 할당 실패 시 NULL 반환
//    pf->start_node = start; // 시작 노드 설정
//    pf->goal_node = goal; // 목표 노드 설정
//    pq_init(&pf->pq, GRID_WIDTH * GRID_HEIGHT); // 우선순위 큐 초기화
//    return pf; // 생성된 경로 탐색기 반환
//}
//
///**
// * @brief 경로 탐색기(Pathfinder)의 메모리를 해제합니다.
// */
//void pathfinder_destroy(Pathfinder* pf) {
//    if (pf) { // 경로 탐색기가 유효하다면
//        pq_free(&pf->pq); // 우선순위 큐 메모리 해제
//        free(pf); // 경로 탐색기 자체 메모리 해제
//    }
//}
//
///**
// * @brief LPA* 알고리즘의 메인 로직을 수행하여 최단 경로를 계산합니다.
// */
//void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, AgentManager* agent_manager) {
//    // 그리드 내 모든 노드의 g, rhs 값을 초기화합니다.
//    for (int y = 0; y < GRID_HEIGHT; y++) {
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            Node* n = &map->grid[y][x];
//            n->g = INF; n->rhs = INF; n->in_pq = FALSE; n->pq_index = -1;
//        }
//    }
//
//    if (pf->goal_node) { // 목표 노드가 설정되어 있다면
//        pf->goal_node->rhs = 0; // 목표 노드의 rhs를 0으로 설정
//        pf->goal_node->key = calculate_key(pf, pf->goal_node); // 키 값을 계산
//        pq_push(&pf->pq, pf->goal_node); // 우선순위 큐에 목표 노드를 추가
//    }
//
//    int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 }; // 상하좌우 이동
//
//    // 큐가 비어있지 않고, 시작 노드의 g, rhs가 일치하지 않거나, 큐의 top key가 시작 노드의 key보다 작을 동안 반복
//    while (pf->pq.size > 0 &&
//        (compare_keys(pq_top_key(&pf->pq), calculate_key(pf, pf->start_node)) < 0 ||
//            fabs(pf->start_node->rhs - pf->start_node->g) > 1e-9)) {
//
//        Node* u = pq_pop(&pf->pq); // 큐에서 우선순위가 가장 높은 노드를 꺼냄
//        if (u->g > u->rhs) { // overconsistent 상태일 때 (더 나은 경로 발견)
//            u->g = u->rhs; // g값을 rhs값으로 업데이트 (consistent 상태로 만듦)
//            for (int i = 0; i < NUM_DIRECTIONS; i++) { // 모든 이웃(predecessor)에 대해
//                int px = u->x + dx[i], py = u->y + dy[i];
//                if (grid_is_valid_coord(px, py)) path_update_vertex(pf, map, agent_manager, &map->grid[py][px]); // 이웃 노드 업데이트
//            }
//        }
//        else { // underconsistent 상태일 때 (기존 경로에 문제 발생)
//            u->g = INF; // g값을 무한대로 설정
//            path_update_vertex(pf, map, agent_manager, u); // 자기 자신을 먼저 업데이트
//            for (int i = 0; i < NUM_DIRECTIONS; i++) { // 모든 이웃에 대해
//                int px = u->x + dx[i], py = u->y + dy[i];
//                if (grid_is_valid_coord(px, py)) path_update_vertex(pf, map, agent_manager, &map->grid[py][px]); // 이웃 노드 업데이트
//            }
//        }
//    }
//}
//
///**
// * @brief 계산된 경로(g값)를 바탕으로 현재 위치에서 이동할 다음 노드를 결정합니다.
// */
//Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* agent_manager, Node* current_node) {
//    // 목표가 없거나, 현재 노드에서 목표까지의 경로가 없거나, 이미 목표에 도달했다면
//    if (!pf->goal_node || current_node->g >= INF || current_node == pf->goal_node) {
//        return current_node; // 현재 위치에 머무름
//    }
//
//    double min_g = INF; // 이웃 중 가장 작은 g값을 저장할 변수
//    Node* best_next_node = current_node; // 최적의 다음 노드, 기본값은 현재 노드
//    double min_dist_to_goal = euclidean_distance(current_node, pf->goal_node); // 목표까지의 현재 거리
//
//    int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 }; // 상하좌우 이동
//
//    for (int i = 0; i < NUM_DIRECTIONS; i++) { // 4방향의 이웃에 대해
//        int nx = current_node->x + dx[i], ny = current_node->y + dy[i];
//        if (grid_is_valid_coord(nx, ny)) { // 유효한 좌표라면
//            Node* neighbor = &map->grid[ny][nx]; // 이웃 노드 포인터
//            if (grid_is_node_blocked(map, agent_manager, neighbor)) continue; // 막힌 곳이면 건너뜀
//
//            if (neighbor->g < min_g) { // 이웃의 g값이 현재 최소 g값보다 작으면 (더 좋은 경로)
//                min_g = neighbor->g; // 최소 g값 갱신
//                best_next_node = neighbor; // 최적의 다음 노드를 이 이웃으로 설정
//                min_dist_to_goal = euclidean_distance(neighbor, pf->goal_node); // 목표까지의 거리도 갱신
//            }
//            // g값이 같은 경우, 목표까지의 유클리드 거리가 더 짧은 쪽을 선택 (tie-breaking)
//            else if (fabs(neighbor->g - min_g) < 1e-9) {
//                double dist_to_goal = euclidean_distance(neighbor, pf->goal_node);
//                if (dist_to_goal < min_dist_to_goal) {
//                    best_next_node = neighbor;
//                    min_dist_to_goal = dist_to_goal;
//                }
//            }
//        }
//    }
//    return best_next_node; // 결정된 최적의 다음 노드 반환
//}
//
//
//// =============================================================================
//// --- 9. 에이전트 관리 및 로직 구현 (AgentManager) ---
//// =============================================================================
///**
// * @brief AgentManager 객체를 생성하고 초기화합니다.
// */
//AgentManager* agent_manager_create() {
//    AgentManager* manager = (AgentManager*)calloc(1, sizeof(AgentManager)); // 메모리 할당 및 0으로 초기화
//    if (!manager) { // 할당 실패 시
//        perror("AgentManager 할당 실패");
//        exit(1);
//    }
//    for (int i = 0; i < MAX_AGENTS; i++) { // 각 에이전트 초기화
//        manager->agents[i].id = i; // ID 설정
//        manager->agents[i].symbol = 'A' + i; // 심볼 설정 ('A', 'B', 'C')
//        manager->agents[i].state = IDLE; // 초기 상태는 '대기'
//    }
//    return manager; // 생성된 manager 객체 포인터 반환
//}
//
///**
// * @brief AgentManager 객체의 메모리를 해제합니다.
// */
//void agent_manager_destroy(AgentManager* manager) {
//    if (manager) free(manager); // manager가 NULL이 아니면 메모리 해제
//}
//
///**
// * @brief 특정 에이전트가 특정 목표까지의 경로 비용(g값)을 계산합니다.
// */
//static double calculate_path_cost(Agent* agent, Node* goal, GridMap* map, AgentManager* agent_manager) {
//    Pathfinder* pf = pathfinder_create(agent->pos, goal); // 경로 탐색기 생성
//    if (!pf) return INF; // 생성 실패 시 무한대 비용 반환
//    pathfinder_compute_shortest_path(pf, map, agent_manager); // 경로 계산
//    double cost = agent->pos->g; // 시작 위치의 g값이 바로 비용
//    pathfinder_destroy(pf); // 경로 탐색기 메모리 해제
//    return cost; // 계산된 비용 반환
//}
//
///**
// * @brief 현재 에이전트에게 가장 효율적인(가까운) 주차 공간을 선택합니다.
// */
//static Node* select_best_parking_spot(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
//    double best_cost = INF; // 최적 비용을 무한대로 초기화
//    Node* best_goal = NULL; // 최적 목표를 NULL로 초기화
//    for (int j = 0; j < map->num_goals; j++) { // 모든 주차 공간에 대해
//        Node* g = map->goals[j]; // 주차 공간 노드 포인터
//        // 이미 주차되어 있거나 다른 에이전트가 예약했다면 건너뜀
//        if (g->is_parked || (g->reserved_by_agent != -1 && g->reserved_by_agent != agent->id)) continue;
//
//        double cost = calculate_path_cost(agent, g, map, agent_manager); // 해당 주차 공간까지의 비용 계산
//        if (cost < best_cost) { // 현재까지의 최적 비용보다 저렴하면
//            best_cost = cost; // 최적 비용 갱신
//            best_goal = g; // 최적 목표 갱신
//        }
//    }
//    if (best_goal) { // 최적 목표를 찾았다면
//        logger_log(logger, "[%sPlan%s] Agent %c, 주차 공간 (%d,%d) 선택 (비용: %.1f)", C_CYN, C_NRM, agent->symbol, best_goal->x, best_goal->y, best_cost);
//    }
//    return best_goal; // 찾은 최적의 주차 공간 반환
//}
//
///**
// * @brief 현재 에이전트에게 가장 효율적인(가까운) 출차할 차량을 선택합니다.
// */
//static Node* select_best_parked_car(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
//    double best_cost = INF; // 최적 비용을 무한대로 초기화
//    Node* best_spot = NULL; // 최적 목표를 NULL로 초기화
//    for (int j = 0; j < map->num_goals; j++) { // 모든 주차 공간에 대해
//        Node* g = map->goals[j]; // 주차 공간 노드 포인터
//        // 주차된 차가 없거나 다른 에이전트가 예약했다면 건너뜀
//        if (!g->is_parked || (g->reserved_by_agent != -1 && g->reserved_by_agent != agent->id)) continue;
//
//        g->is_parked = FALSE; // 경로 계산을 위해 '주차됨' 상태를 임시로 해제
//        double cost = calculate_path_cost(agent, g, map, agent_manager); // 비용 계산
//        g->is_parked = TRUE;  // 원래 상태로 복원
//
//        if (cost < best_cost) { // 더 저렴한 비용을 찾으면
//            best_cost = cost; // 최적 비용 갱신
//            best_spot = g; // 최적 목표 갱신
//        }
//    }
//    if (best_spot) { // 최적 목표를 찾았다면
//        logger_log(logger, "[%sPlan%s] Agent %c, 출차 차량 (%d,%d) 선택 (비용: %.1f)", C_CYN, C_NRM, agent->symbol, best_spot->x, best_spot->y, best_cost);
//    }
//    return best_spot; // 찾은 최적의 출차 차량 위치 반환
//}
//
///**
// * @brief 현재 에이전트에게 가장 효율적인(가까운) 충전소를 선택합니다.
// */
//static Node* select_best_charge_station(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
//    double best_cost = INF; // 최적 비용 초기화
//    Node* best_station = NULL; // 최적 충전소 초기화
//    for (int i = 0; i < map->num_charge_stations; i++) { // 모든 충전소에 대해
//        Node* station = map->charge_stations[i]; // 충전소 노드 포인터
//        // 다른 에이전트가 예약했다면 건너뜀
//        if (station->reserved_by_agent != -1 && station->reserved_by_agent != agent->id) continue;
//        double cost = calculate_path_cost(agent, station, map, agent_manager); // 비용 계산
//        if (cost < best_cost) { // 더 저렴한 비용을 찾으면
//            best_cost = cost; // 비용 갱신
//            best_station = station; // 충전소 갱신
//        }
//    }
//    if (best_station) { // 최적 충전소를 찾았다면
//        logger_log(logger, "[%sPlan%s] Agent %c, 충전소 (%d,%d) 선택 (비용: %.1f)", C_CYN, C_NRM, agent->symbol, best_station->x, best_station->y, best_cost);
//    }
//    return best_station; // 찾은 최적의 충전소 반환
//}
//
///**
// * @brief 에이전트의 현재 상태에 따라 목표(goal)를 설정합니다.
// */
//static void agent_set_goal(Agent* agent, GridMap* map, AgentManager* agent_manager, Logger* logger) {
//    // '빈 차로 복귀 중'일 때만 충전이 필요하면 즉시 경로를 변경합니다.
//    // 차량을 운반 중(주차하러 가거나, 출차시키러 가는 중)일 때는 현재 작업을 완료해야 하므로, 이 경우에는 경로를 변경하지 않습니다.
//    if (agent->state == RETURNING_HOME_EMPTY && agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
//        if (agent->goal) { // 이미 기지로 향하는 목표가 설정되어 있었다면
//            agent->goal->reserved_by_agent = -1; // 이전 목표의 예약을 해제
//            agent->goal = NULL; // 목표를 초기화하여 새로운 목표를 설정할 수 있도록 함
//        }
//        logger_log(logger, "[%sCharge%s] Agent %c 충전 필요! 목표를 충전소로 즉시 변경.", C_B_YEL, C_NRM, agent->symbol);
//        agent->state = GOING_TO_CHARGE; // 상태를 충전소로 가는 상태로 변경
//    }
//
//    // 대기 중이거나, 충전 중이거나, 이미 목표가 설정된 경우엔 새로운 목표를 설정하지 않음
//    if (agent->state == IDLE || agent->state == CHARGING || agent->goal != NULL) return;
//
//    switch (agent->state) { // 에이전트 상태에 따라
//    case GOING_TO_PARK: agent->goal = select_best_parking_spot(agent, map, agent_manager, logger); break; // 최적 주차 공간 선택
//    case RETURNING_HOME_EMPTY:
//    case RETURNING_WITH_CAR:
//    case RETURNING_HOME_MAINTENANCE:
//        agent->goal = agent->home_base; break; // 목표를 기지로 설정
//    case GOING_TO_COLLECT: agent->goal = select_best_parked_car(agent, map, agent_manager, logger); break; // 최적 출차 차량 선택
//    case GOING_TO_CHARGE: agent->goal = select_best_charge_station(agent, map, agent_manager, logger); break; // 최적 충전소 선택
//    default: break;
//    }
//
//    if (agent->goal) { // 목표가 성공적으로 설정되었다면
//        agent->goal->reserved_by_agent = agent->id; // 해당 목표를 현재 에이전트가 예약했다고 표시
//    }
//    else if (agent->state != RETURNING_HOME_EMPTY && agent->state != RETURNING_WITH_CAR && agent->state != RETURNING_HOME_MAINTENANCE) { // 목표 설정에 실패했고, 복귀 중이 아니라면
//        agent->state = IDLE; // 대기 상태로 전환
//        logger_log(logger, "[%sInfo%s] Agent %c: 가용 목표 없음. 대기 상태로 전환.", C_YEL, C_NRM, agent->symbol);
//    }
//}
//
///**
// * @brief 모든 에이전트의 다음 이동을 계획하고, 발생할 수 있는 충돌을 해결합니다.
// */
//void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
//    // --- 1단계: 목표 선정 ---
//    // 모든 에이전트는 임시 장애물이 없는 '깨끗한' 맵을 기준으로 이상적인 목표를 선택합니다.
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        // 에이전트가 새 목표를 받을 준비가 된 경우에만 목표를 설정합니다. (예: 기존 목표가 없는 경우)
//        if (agent->goal == NULL && agent->state != IDLE && agent->state != CHARGING) {
//            agent_set_goal(agent, map, manager, logger);
//        }
//    }
//
//    // 다음 위치를 현재 위치로 초기화
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        next_pos[i] = manager->agents[i].pos;
//    }
//
//    // --- 2단계: 경로 계획 및 충돌 회피 ---
//    // 각 에이전트는 선택된 목표를 향한 다음 한 걸음을 계획하며, 이 과정에서 다른 에이전트를 피합니다.
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        if (agent->state == IDLE || agent->state == CHARGING || agent->goal == NULL) {
//            continue; // 비활성 상태이거나 목표가 없는 에이전트는 건너뜁니다.
//        }
//
//        // 이 에이전트의 경로 탐색을 위해 다른 에이전트들을 임시 장애물로 설정
//        Node* obstacles_to_clear[MAX_AGENTS];
//        int obs_count = 0;
//        for (int j = 0; j < MAX_AGENTS; j++) {
//            if (i == j) continue;
//            // 우선순위가 높은(먼저 계산된) 에이전트는 이미 계산된 다음 위치를, 아닌 경우는 현재 위치를 장애물로 설정
//            Node* obs_node = (j < i) ? next_pos[j] : manager->agents[j].pos;
//            if (obs_node) {
//                obs_node->is_temp = TRUE;
//                obstacles_to_clear[obs_count++] = obs_node;
//            }
//        }
//
//        // 목표를 재선정하지 않고 다음 스텝만 계획
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
//        // 임시 장애물 설정 해제
//        for (int k = 0; k < obs_count; k++) {
//            if (obstacles_to_clear[k]) obstacles_to_clear[k]->is_temp = FALSE;
//        }
//    }
//
//    // --- 3단계: 충돌 해결 ---
//    // 계산된 다음 스텝들을 기반으로 최종 충돌을 해결합니다.
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        for (int j = i + 1; j < MAX_AGENTS; j++) {
//            if (manager->agents[i].state == IDLE || manager->agents[j].state == IDLE ||
//                manager->agents[i].state == CHARGING || manager->agents[j].state == CHARGING) continue;
//
//            // 충돌 감지 1: 두 에이전트가 같은 다음 위치로 이동하려는 경우
//            if (next_pos[i] == next_pos[j]) {
//                logger_log(logger, "[%sAvoid%s] 충돌 감지! Agent %c 대기.", C_B_RED, C_NRM, manager->agents[j].symbol);
//                next_pos[j] = manager->agents[j].pos; // 우선순위가 낮은 j가 현재 위치에 대기
//            }
//            // 충돌 감지 2: 두 에이전트가 서로의 위치로 맞바꾸려 하는 경우 (교차 충돌)
//            else if (next_pos[i] == manager->agents[j].pos && next_pos[j] == manager->agents[i].pos) {
//                logger_log(logger, "[%sAvoid%s] 교차 충돌 감지! Agent %c 대기.", C_B_RED, C_NRM, manager->agents[j].symbol);
//                next_pos[j] = manager->agents[j].pos; // 우선순위가 낮은 j가 현재 위치에 대기
//            }
//        }
//    }
//}
//
///**
// * @brief 에이전트가 이동을 마친 후 상태를 업데이트합니다. (목표 도달 처리)
// */
//void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, Logger* logger) {
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        // 비활성 상태이거나, 목표가 없거나, 아직 목표에 도달하지 못했다면 건너뜀
//        if (agent->state == IDLE || agent->state == CHARGING || !agent->goal || agent->pos != agent->goal) {
//            continue;
//        }
//
//        Node* reached_goal = agent->goal; // 도달한 목표 노드
//
//        if (agent->state != GOING_TO_CHARGE) {
//            reached_goal->reserved_by_agent = -1;
//        }
//        agent->goal = NULL; // 에이전트의 목표 초기화
//
//        switch (agent->state) {
//        case GOING_TO_PARK: // 주차하러 가는 중이었다면
//            reached_goal->is_parked = TRUE;
//            manager->total_cars_parked++;
//            logger_log(logger, "[%sPark%s] Agent %c, 주차 완료 at (%d,%d).", C_GRN, C_NRM, agent->symbol, reached_goal->x, reached_goal->y);
//
//            // [Task Count Fix] 주차 완료 시점에 바로 작업 카운트를 증가시킵니다.
//            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == PARK_PHASE) {
//                scenario->tasks_completed_in_phase++;
//            }
//
//            agent->state = RETURNING_HOME_EMPTY;
//            break;
//
//        case RETURNING_HOME_EMPTY: // 주차 후 기지로 복귀 중이었다면
//            logger_log(logger, "[%sInfo%s] Agent %c, 주차 작업 후 기지 복귀 완료.", C_CYN, C_NRM, agent->symbol);
//            // 작업 카운트는 주차 시점에 이미 완료되었으므로 여기서는 아무것도 하지 않습니다.
//            agent->state = IDLE;
//            break;
//
//        case GOING_TO_COLLECT:
//            logger_log(logger, "[%sExit%s] Agent %c, 차량 수거 at (%d,%d).", C_YEL, C_NRM, agent->symbol, reached_goal->x, reached_goal->y);
//            reached_goal->is_parked = FALSE;
//            manager->total_cars_parked--;
//            agent->state = RETURNING_WITH_CAR;
//            break;
//
//        case RETURNING_WITH_CAR: // 차를 싣고 복귀 중이었다면
//            logger_log(logger, "[%sExit%s] Agent %c, 차량 출차 완료.", C_GRN, C_NRM, agent->symbol);
//
//            // [Task Count Confirmed] 출차 단계였다면, 기지 도착 시점에 작업 완료 카운트를 올립니다.
//            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases && scenario->phases[scenario->current_phase_index].type == EXIT_PHASE) {
//                scenario->tasks_completed_in_phase++;
//            }
//            agent->state = IDLE;
//            break;
//
//        case GOING_TO_CHARGE:
//            logger_log(logger, "[%sCharge%s] Agent %c, 충전 시작. (%d steps)", C_B_YEL, C_NRM, agent->symbol, CHARGE_TIME);
//            agent->state = CHARGING;
//            agent->charge_timer = CHARGE_TIME;
//            break;
//
//        case RETURNING_HOME_MAINTENANCE:
//            logger_log(logger, "[%sInfo%s] Agent %c, 충전 후 기지 복귀 완료.", C_CYN, C_NRM, agent->symbol);
//            agent->state = IDLE;
//            break;
//
//        default: break;
//        }
//    }
//}
//
///**
// * @brief 충전 중인 에이전트의 상태를 업데이트합니다.
// */
//void agent_manager_update_charge_state(AgentManager* manager, Logger* logger) {
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        if (agent->state == CHARGING) { // 충전 상태인 에이전트에 대해
//            agent->charge_timer--; // 충전 타이머 감소
//            if (agent->charge_timer <= 0) { // 타이머가 0 이하가 되면 충전 완료
//                logger_log(logger, "[%sCharge%s] Agent %c 충전 완료.", C_B_GRN, C_NRM, agent->symbol);
//                agent->total_distance_traveled = 0.0; // 이동 거리 초기화
//                agent->state = RETURNING_HOME_MAINTENANCE;
//                if (agent->pos) agent->pos->reserved_by_agent = -1; // 현재 위치(충전소)의 예약 해제
//                agent->goal = NULL; // 목표 초기화
//            }
//        }
//    }
//}
//
//// =============================================================================
//// --- 10. 시뮬레이션 코어 로직 구현 ---
//// =============================================================================
///**
// * @brief 전체 시뮬레이션 객체(Simulation)를 생성하고 하위 모듈들을 초기화합니다.
// */
//Simulation* simulation_create() {
//    Simulation* sim = (Simulation*)calloc(1, sizeof(Simulation)); // Simulation 구조체 메모리 할당 및 0으로 초기화
//    if (!sim) { perror("Simulation 할당 실패"); exit(1); } // 할당 실패 시 오류 처리
//
//    sim->agent_manager = agent_manager_create(); // 에이전트 매니저 생성
//    sim->map = grid_map_create(sim->agent_manager); // 맵 생성 (에이전트 초기 위치 설정을 위해 agent_manager 필요)
//    sim->scenario_manager = scenario_manager_create(); // 시나리오 매니저 생성
//    sim->logger = logger_create(); // 로거 생성
//    return sim; // 생성된 시뮬레이션 객체 포인터 반환
//}
//
///**
// * @brief 시뮬레이션 객체와 모든 하위 모듈의 메모리를 해제합니다.
// */
//void simulation_destroy(Simulation* sim) {
//    if (sim) { // 시뮬레이션 객체가 유효하다면
//        grid_map_destroy(sim->map); // 맵 메모리 해제
//        agent_manager_destroy(sim->agent_manager); // 에이전트 매니저 메모리 해제
//        scenario_manager_destroy(sim->scenario_manager); // 시나리오 매니저 메모리 해제
//        logger_destroy(sim->logger); // 로거 메모리 해제
//        free(sim); // 시뮬레이션 객체 자체 메모리 해제
//    }
//}
//
///**
// * @brief 시뮬레이션의 상태를 한 스텝 업데이트합니다. (작업 할당 등)
// */
//static void simulation_update_state(Simulation* sim) {
//    ScenarioManager* scenario = sim->scenario_manager;
//    AgentManager* agent_manager = sim->agent_manager;
//    GridMap* map = sim->map;
//    Logger* logger = sim->logger;
//
//    // --- 사용자 정의 시나리오 모드 로직 ---
//    if (scenario->mode == MODE_CUSTOM) {
//        if (scenario->current_phase_index >= scenario->num_phases) return; // 모든 단계가 완료되었으면 종료
//
//        DynamicPhase* phase = &scenario->phases[scenario->current_phase_index]; // 현재 단계 정보
//        // 현재 단계에서 요구하는 작업 수가 모두 완료되었다면
//        if (scenario->tasks_completed_in_phase >= phase->task_count) {
//            logger_log(logger, "[%sPhase%s] %d단계 (%s %d대) 완료!", C_B_YEL, C_NRM, scenario->current_phase_index + 1, phase->type_name, phase->task_count);
//            scenario->current_phase_index++; // 다음 단계로 이동
//            scenario->tasks_completed_in_phase = 0; // 완료된 작업 수 초기화
//            if (scenario->current_phase_index < scenario->num_phases) { // 다음 단계가 있다면
//                DynamicPhase* next_phase = &scenario->phases[scenario->current_phase_index]; // 다음 단계 정보
//                logger_log(logger, "[%sPhase%s] %d단계 시작: %s %d대.", C_B_YEL, C_NRM, scenario->current_phase_index + 1, next_phase->type_name, next_phase->task_count);
//                sleep(1500); // 단계 전환 시 잠시 대기
//            }
//            return;
//        }
//    }
//    // --- 실시간 시뮬레이션 모드 로직 ---
//    else if (scenario->mode == MODE_REALTIME) {
//        // 일정 시간 간격마다 새로운 작업(이벤트)을 생성
//        if (scenario->time_step > 0 && scenario->time_step % EVENT_GENERATION_INTERVAL == 0) {
//            int event_chance = rand() % 100; // 0~99 사이의 난수 생성
//            // 주차 확률에 해당하고, 주차 공간이 남아있으면
//            if (event_chance < scenario->park_chance && agent_manager->total_cars_parked < map->num_goals) {
//                logger_log(logger, "[%sEvent%s] 새로운 주차 요청 발생.", C_B_GRN, C_NRM);
//                add_task_to_queue(scenario, TASK_PARK); // 주차 작업을 큐에 추가
//            }
//            // 출차 확률에 해당하고, 주차된 차가 있으면
//            else if (event_chance < (scenario->park_chance + scenario->exit_chance) && agent_manager->total_cars_parked > 0) {
//                logger_log(logger, "[%sEvent%s] 새로운 출차 요청 발생.", C_B_YEL, C_NRM);
//                add_task_to_queue(scenario, TASK_EXIT); // 출차 작업을 큐에 추가
//            }
//        }
//    }
//
//    // --- 모든 에이전트에 대한 작업 할당 로직 ---
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &agent_manager->agents[i];
//        if (agent->state == IDLE) { // 대기 중인 에이전트에게만 작업 할당 시도
//            // 충전이 필요한지 먼저 확인
//            if (agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
//                if (select_best_charge_station(agent, map, agent_manager, logger)) { // 가용한 충전소가 있다면
//                    agent->state = GOING_TO_CHARGE; // 충전소로 가는 상태로 변경
//                }
//                else { // 모든 충전소가 사용 중이면
//                    logger_log(logger, "[%sWarn%s] Agent %c 충전 필요하나 모든 충전소가 사용 중.", C_YEL, C_NRM, agent->symbol);
//                }
//                continue; // 충전이 우선이므로 다른 작업 할당은 건너뜀
//            }
//
//            // [커스텀 모드 작업 할당]
//            if (scenario->mode == MODE_CUSTOM) {
//                if (scenario->current_phase_index >= scenario->num_phases) continue; // 모든 단계 완료 시 건너뜀
//                DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
//
//                if (phase->type == PARK_PHASE) { // 현재가 주차 단계일 경우
//                    int active_tasks = 0; // 현재 주차 작업을 수행 중인 에이전트 수
//                    for (int j = 0; j < MAX_AGENTS; j++) {
//                        AgentState s = agent_manager->agents[j].state;
//                        if (s == GOING_TO_PARK || s == RETURNING_HOME_EMPTY) {
//                            active_tasks++;
//                        }
//                    }
//                    // (완료된 작업 + 진행 중인 작업)이 목표량보다 적고, 주차 공간이 남았다면
//                    if ((scenario->tasks_completed_in_phase + active_tasks) < phase->task_count && agent_manager->total_cars_parked < map->num_goals) {
//                        agent->state = GOING_TO_PARK; // 주차 작업 할당
//                        logger_log(logger, "[%sTask%s] Agent %c, 신규 주차 작업 할당.", C_CYN, C_NRM, agent->symbol);
//                    }
//                }
//                else if (phase->type == EXIT_PHASE) { // 현재가 출차 단계일 경우
//                    int active_tasks = 0; // 현재 출차 작업을 수행 중인 에이전트 수
//                    for (int j = 0; j < MAX_AGENTS; j++) {
//                        AgentState s = agent_manager->agents[j].state;
//                        if (s == GOING_TO_COLLECT || s == RETURNING_WITH_CAR) {
//                            active_tasks++;
//                        }
//                    }
//                    // (완료된 작업 + 진행 중인 작업)이 목표량보다 적고, 출차할 차가 있다면
//                    if ((scenario->tasks_completed_in_phase + active_tasks) < phase->task_count && agent_manager->total_cars_parked > 0) {
//                        agent->state = GOING_TO_COLLECT; // 출차 작업 할당
//                        logger_log(logger, "[%sTask%s] Agent %c, 신규 출차 작업 할당.", C_CYN, C_NRM, agent->symbol);
//                    }
//                }
//            }
//            // [실시간 모드 작업 할당 - 연결 리스트 기반]
//            else if (scenario->mode == MODE_REALTIME && scenario->task_count > 0) {
//                int is_parking_lot_full = (agent_manager->total_cars_parked >= map->num_goals);
//                TaskNode* current = scenario->task_queue_head;
//                TaskNode* prev = NULL;
//                int task_assigned = FALSE;
//
//                while (current != NULL) {
//                    int can_process = FALSE;
//                    // 주차장이 꽉 찼을 경우: 출차 요청만 찾음
//                    if (is_parking_lot_full) {
//                        if (current->type == TASK_EXIT && agent_manager->total_cars_parked > 0) {
//                            can_process = TRUE;
//                        }
//                    }
//                    // 주차장이 꽉 차지 않았을 경우: 모든 유효한 요청을 순서대로 처리
//                    else {
//                        if (current->type == TASK_PARK) {
//                            can_process = TRUE; // 주차 공간이 있으므로 주차 요청 처리 가능
//                        }
//                        else if (current->type == TASK_EXIT && agent_manager->total_cars_parked > 0) {
//                            can_process = TRUE; // 출차할 차가 있으므로 출차 요청 처리 가능
//                        }
//                    }
//
//                    if (can_process) {
//                        // 작업 할당
//                        if (current->type == TASK_PARK) {
//                            agent->state = GOING_TO_PARK;
//                            logger_log(logger, "[%sTask%s] Agent %c, 주차 작업 할당.", C_CYN, C_NRM, agent->symbol);
//                        }
//                        else { // TASK_EXIT
//                            agent->state = GOING_TO_COLLECT;
//                            logger_log(logger, "[%sTask%s] Agent %c, 출차 작업 할당.", C_CYN, C_NRM, agent->symbol);
//                        }
//                        task_assigned = TRUE;
//
//                        // 큐에서 해당 노드 제거
//                        if (prev == NULL) { // 제거할 노드가 헤드일 경우
//                            scenario->task_queue_head = current->next;
//                        }
//                        else {
//                            prev->next = current->next;
//                        }
//                        if (current == scenario->task_queue_tail) { // 제거할 노드가 테일일 경우
//                            scenario->task_queue_tail = prev;
//                        }
//                        free(current);
//                        scenario->task_count--;
//                        break; // 이 에이전트에 대한 작업 할당 완료
//                    }
//
//                    // 다음 노드로 이동
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
// * @brief 시뮬레이션이 완료되었는지 확인합니다.
// */
//static int simulation_is_complete(const Simulation* sim) {
//    const ScenarioManager* scenario = sim->scenario_manager;
//    const AgentManager* agent_manager = sim->agent_manager;
//
//    // 커스텀 모드: 모든 단계가 끝나고, 모든 에이전트가 대기 상태일 때 완료
//    if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index >= scenario->num_phases) {
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            if (agent_manager->agents[i].state != IDLE) return FALSE; // 아직 활동 중인 에이전트가 있으면 미완료
//        }
//        printf(C_B_GRN "\n모든 시나리오 단계가 완료되었습니다! 시뮬레이션을 종료합니다.\n" C_NRM);
//        return TRUE; // 완료
//    }
//
//    // 실시간 모드: 설정된 시간 제한에 도달했을 때 완료
//    if (scenario->mode == MODE_REALTIME && scenario->time_step >= REALTIME_MODE_TIMELIMIT) {
//        printf(C_B_GRN "\n실시간 시뮬레이션 시간 제한에 도달했습니다! 시뮬레이션을 종료합니다.\n" C_NRM);
//        return TRUE; // 완료
//    }
//    return FALSE; // 미완료
//}
//
///**
// * @brief 메인 시뮬레이션 루프를 실행합니다.
// */
//void simulation_run(Simulation* sim) {
//    while (TRUE) { // 무한 루프
//        // 상태 업데이트 및 로직 계산
//        agent_manager_update_charge_state(sim->agent_manager, sim->logger); // 1. 충전 상태 업데이트
//        simulation_update_state(sim); // 2. 작업 할당 등 전체 상태 업데이트
//
//        Node* next_pos[MAX_AGENTS]; // 각 에이전트의 다음 위치를 저장할 배열
//        agent_manager_plan_and_resolve_collisions(sim->agent_manager, sim->map, sim->logger, next_pos); // 3. 경로 계획 및 충돌 해결
//
//        // 계산된 다음 위치로 에이전트 이동
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            if (sim->agent_manager->agents[i].state != CHARGING && next_pos[i] != NULL) { // 충전 중이 아닐 때만 이동
//                if (sim->agent_manager->agents[i].pos != next_pos[i]) { // 실제로 이동했다면
//                    sim->agent_manager->agents[i].total_distance_traveled += 1.0; // 이동 거리 증가
//                }
//                sim->agent_manager->agents[i].pos = next_pos[i]; // 위치 업데이트
//            }
//        }
//        agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->logger); // 4. 이동 후 목표 도달 등 상태 업데이트
//
//        // 최적화된 렌더링 함수 호출
//        simulation_display_status(sim); // 5. 현재 상태를 화면에 표시
//
//        if (simulation_is_complete(sim)) break; // 6. 시뮬레이션 완료 조건 확인 후 루프 탈출
//
//        sim->scenario_manager->time_step++; // 시간 스텝 증가
//        sleep(sim->scenario_manager->simulation_speed); // 설정된 속도만큼 대기
//    }
//}
//
//
//// =============================================================================
//// --- 11. 메인 함수 ---
//// =============================================================================
//int main() {
//    srand((unsigned int)time(NULL)); // 난수 생성기 시드 설정
//    system_enable_virtual_terminal(); // Windows에서 ANSI 컬러 코드 활성화
//
//    Simulation* sim = simulation_create(); // 시뮬레이션 객체 생성
//    if (!sim) return 1; // 생성 실패 시 종료
//
//    if (simulation_setup(sim)) { // 사용자로부터 시뮬레이션 설정을 받고 성공하면
//        simulation_run(sim); // 시뮬레이션 메인 루프 실행
//    }
//    else { // 설정이 취소되면
//        printf("\n시뮬레이션이 취소되었습니다. 프로그램을 종료합니다.\n");
//    }
//
//    simulation_destroy(sim); // 할당된 모든 메모리 해제
//    return 0; // 프로그램 정상 종료
//}
