#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>

// System headers
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <getopt.h> 

#include "KC_heap.hpp"
#include "KC_searching.hpp"
#include "KC_structs.hpp"
#include "KC_astar.hpp"
#include "KC_algorithms.hpp"
#include "common.hpp"
#include "rassert.hpp"

using namespace std;

// Global memory pool
MyHEAP *HEAP = nullptr;

// =============================================================================
// BENCHMARK CONFIGURATION (HARDCODED FOR FORK MODE)
// =============================================================================

// List of maps to test (files must exist in maps/ directory)
vector<string> maps = {
    "Moscow_0_512", 
    //"Labyrinth", 
    //"Milan_1_256"
};

// Control sets (files must exist in data/ directory)
vector<string> cs = {
    "base_control_set",
};

// Corresponding Mesh Graph information (must match indices of 'cs')
vector<string> meshes = {
    "base_mesh_info",
};

// Heuristic weights to test
vector<float> ww = {1.0,};

// Goal tolerance constants
const float GLOBAL_R = 0.0;
const int GLOBAL_A = 0;


// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

static void load_scenarios(vector<Vertex*> &starts, vector<Vertex*> &goals, string scen_file, int samples = 3) {
    /*
     Parses testing scenarios from a standard MovingAI .scen file.
     The file specifies start and goal grid coordinates (i, j). For each entry in the file, 'samples' test
     instances are generated. The start and goal headings for each instance are sampled uniformly and 
     independently from [0, ANGLE_NUM-1].
    */
    std::random_device dev;
    std::mt19937 rng(dev());
    rng.seed(12345);
    std::uniform_int_distribution<std::mt19937::result_type> dist_theta(0, NUM_HEADINGS - 1);

    ifstream file(scen_file);
    if (!file.is_open()) {
        cerr << "Warning: Cannot open scenario file " << scen_file << ". Benchmarks might fail." << endl;
        return;
    }
    
    string line, temp;
    getline(file, temp); // Skip version header

    while (getline(file, line)) {
        stringstream stream(line);
        int start_i, start_j, goal_i, goal_j;
        // MovingAI format: bucket map width height start_x start_y goal_x goal_y dist
        stream >> temp >> temp >> temp >> temp >> start_j >> start_i >> goal_j >> goal_i;
        
        for (int k = 0; k < samples; k++) {
            int start_theta = dist_theta(rng);
            int goal_theta = dist_theta(rng);
            starts.push_back(new Vertex(start_i, start_j, start_theta));
            goals.push_back(new Vertex(goal_i, goal_j, goal_theta));
        }
    }
    file.close();
}

void parse_state_string(string s, int &i, int &j, int &theta) {
    stringstream ss(s);
    ss >> i >> j >> theta;
    if (ss.fail()) {
        throw runtime_error("Invalid state string format. Expected 'i j theta', got: " + s);
    }
}

// =============================================================================
// MODE: SINGLE RUN (Detailed Debug)
// =============================================================================

void solve_single_instance(string map_file, string prim_file, string mesh_file, 
                           string start_str, string goal_str, string out_prefix, float w) {
    
    // 1. Initialize
    cout << "Loading data..." << endl;
    Map *map = new Map();
    map->read_file_to_cells(map_file);
    
    ControlSet *control_set = new ControlSet();
    control_set->load_primitives(prim_file);
    
    MeshInfo *mesh_info = new MeshInfo();
    mesh_info->load(mesh_file);

    // 2. Parse Start/Goal
    int si, sj, st, gi, gj, gt;
    parse_state_string(start_str, si, sj, st);
    parse_state_string(goal_str, gi, gj, gt);
    
    Vertex *start = new Vertex(si, sj, st);
    Vertex *finish = new Vertex(gi, gj, gt);

    cout << "Task: " << si << "," << sj << "," << st << " -> " << gi << "," << gj << "," << gt << endl;
    cout << "Heuristic Weight: " << w << endl;

    // 3. Setup Algorithms. We run three variants.
    vector<SearchAlgorithm*> algos;
    
    // a) LazyLBA* (Lattice + Lazy)
    algos.push_back(new SearchAlgorithm("LazyLBA*", map, control_set, nullptr, true, w, GLOBAL_R, GLOBAL_A, ALGO_LATTICE_ASTAR));
    
    // b) LBA* (Lattice + Check immediately)
    algos.push_back(new SearchAlgorithm("LBA*", map, control_set, nullptr, false, w, GLOBAL_R, GLOBAL_A, ALGO_LATTICE_ASTAR));
    
    // c) MeshA* (Mesh + Check immediately, Lazy makes no sense here)
    algos.push_back(new SearchAlgorithm("MeshA*", map, control_set, mesh_info, false, w, GLOBAL_R, GLOBAL_A, ALGO_MESH_ASTAR));

    // 4. Run and Dump
    for (auto algo : algos) {
        cout << "Running " << algo->name << "..." << flush;
        algo->solve(start, finish);
        cout << " Done. Solved: " << algo->is_solved << ", Time: " << algo->total_runtime << "s" << endl;

        // Generate filename: prefix + result_AlgoName.txt
        // e.g., "res/0_result_MeshA*.txt" (we strip '*' for safety usually, but let's keep it simple)
        string clean_name = algo->name;
        if (clean_name == "MeshA*") clean_name = "MeshAstar";
        if (clean_name == "LazyLBA*") clean_name = "LazyLBAstar";
        if (clean_name == "LBA*") clean_name = "LBAstar";

        string filename = out_prefix + "result_" + clean_name + ".txt";
        algo->dump_result_of_search(filename);
    }

    // 5. Cleanup
    for (auto algo : algos) delete algo;
    delete start;
    delete finish;
    delete mesh_info;
    delete control_set;
    delete map;
}

// =============================================================================
// MODE: BENCHMARK (Mass Testing)
// =============================================================================

void run_test_suite(string map_path, string scen_path, string prim_path, string mesh_path, float w, 
                    string out_file) {
    
    // Load resources
    Map *map = new Map();
    map->read_file_to_cells(map_path);

    ControlSet *control_set = new ControlSet();
    control_set->load_primitives(prim_path);

    MeshInfo *mesh_info = new MeshInfo();
    mesh_info->load(mesh_path);

    vector<Vertex*> starts, goals;
    load_scenarios(starts, goals, scen_path);
    int num_tests = min((int)starts.size(), MAX_TESTS);

    ofstream resfile(out_file);
    if (!resfile.is_open()) {
        cerr << "Error writing to " << out_file << endl;
        exit(1);
    }

    resfile << "TOTAL_TESTS: " << num_tests << endl;
    
    // Define algorithms for the suite
    vector<SearchAlgorithm*> algos = {
        new SearchAlgorithm("MeshA*", map, control_set, mesh_info, false, w, GLOBAL_R, GLOBAL_A, ALGO_MESH_ASTAR),
        new SearchAlgorithm("LBA*", map, control_set, nullptr, false, w, GLOBAL_R, GLOBAL_A, ALGO_LATTICE_ASTAR),
        new SearchAlgorithm("LazyLBA*", map, control_set, nullptr, true, w, GLOBAL_R, GLOBAL_A, ALGO_LATTICE_ASTAR)
    };

    for (int i = 0; i < num_tests; i++) {
        resfile << "=== Test " << i << " ===" << endl;
        resfile << "Start: " << starts[i]->i << " " << starts[i]->j << " " << starts[i]->theta << endl;
        resfile << "Goal: " << goals[i]->i << " " << goals[i]->j << " " << goals[i]->theta << endl;
        resfile << "---" << endl;

        for (auto algo : algos) {
            algo->solve(starts[i], goals[i]);

            double len = algo->is_solved ? metric_length(algo->path, control_set) : 0.0;
            double aol = algo->is_solved ? metric_AOL(algo->path, control_set) : 0.0;

            resfile << "Algorithm: " << algo->name << endl;
            resfile << "result (solved, steps, cost): " << (int)algo->is_solved << " " << algo->steps << " " << algo->cost_path << endl;
            resfile << "metrics (length, AOL): " << len << " " << aol << endl;
            resfile << "number of checked cells: " << algo->checked_cells << endl;
            resfile << "time in working: " << algo->total_runtime << endl;
            resfile << "---" << endl;
        }
    }
    resfile.close();

    // Cleanup
    for (auto algo : algos) delete algo;
    delete mesh_info;
    delete control_set;
    delete map;
    for (auto v : starts) delete v;
    for (auto v : goals) delete v;
}

void run_fork_benchmark() {
    string pref_map = "maps/";
    string pref_data = "data/";
    string pref_res = "res/";

    rassert(cs.size() == meshes.size(), "Control sets and Types vectors must have equal length!");

    vector<pid_t> pids;

    for (size_t i = 0; i < maps.size(); i++) {
        for (size_t j = 0; j < cs.size(); j++) {
            for (float w : ww) {
                
                pid_t pid = fork();

                if (pid < 0) {
                    cerr << "Fork failed!" << endl;
                    exit(1);
                }

                if (pid == 0) { // Child Process
                    HEAP = new MyHEAP(); // Own memory pool

                    string map_path = pref_map + maps[i] + ".map";
                    string scen_path = pref_map + maps[i] + ".map.scen";
                    string prim_path = pref_data + cs[j] + ".txt";
                    string mesh_path = pref_data + meshes[j] + ".txt";
                    
                    // Generate Result Filename: res/Map_CS_w_R_A.txt
                    stringstream ss_out;
                    ss_out << pref_res << maps[i] << "_" << meshes[j] << "_" << cs[j]\
                           << "_w" << w << "_R" << GLOBAL_R << "_A" << GLOBAL_A << ".txt";
                    
                    cout << "[Child] Testing: " << maps[i] << " with " << cs[j] << " (w=" << w << ")" << endl;
                    
                    run_test_suite(map_path, scen_path, prim_path, mesh_path, w, 
                                   ss_out.str());

                    delete HEAP;
                    exit(0); // Child exits cleanly
                } else {
                    // Parent
                    pids.push_back(pid);
                }
            }
        }
    }

    cout << "[Parent] All processes launched. Waiting..." << endl;
    int status;
    for (pid_t p : pids) {
        waitpid(p, &status, 0);
    }
    cout << "[Parent] All benchmarks finished." << endl;
}

// =============================================================================
// MAIN & PARSING
// =============================================================================

void print_help() {
    cout << "MeshA* Planner Executable\n";
    cout << "Usage:\n";
    cout << "1. Benchmark Mode (Forked, Hardcoded Config):\n";
    cout << "   ./mesh_astar --mode benchmark\n\n";
    cout << "   Current config (in the start of the file) needs: " << maps.size() * cs.size() * meshes.size() * ww.size() << " processes!\n\n";
    cout << "2. Single Run Mode (Detail Output):\n";
    cout << "   ./mesh_astar --mode single --map <file> --prim <file> --mesh <file> \\\n";
    cout << "                --start \"i j theta\" --goal \"i j theta\" --out-prefix <str> --weight <val>\n";
    cout << "   Example:\n";
    cout << "   ./mesh_astar --mode single --map maps/Moscow.map ... \\\n";
    cout << "                --start \"10 10 0\" --goal \"50 50 4\" --out-prefix \"res/0_\" --weight 1.5\n";
}

int main(int argc, char* argv[]) {
    string mode = "";
    string map_file, prim_file, mesh_file;
    string start_str, goal_str;
    string out_prefix = "res/";
    float w = 1.0;

    static struct option long_options[] = {
        {"mode",       required_argument, 0, 'm'},
        {"map",        required_argument, 0, 'a'},
        {"prim",       required_argument, 0, 'p'},
        {"mesh",       required_argument, 0, 'c'},
        {"start",      required_argument, 0, 's'},
        {"goal",       required_argument, 0, 'g'},
        {"out-prefix", required_argument, 0, 'o'},
        {"weight",     required_argument, 0, 'w'},
        {"help",       no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int opt_idx = 0;
    while (true) {
        int c = getopt_long(argc, argv, "m:h", long_options, &opt_idx);
        if (c == -1) break;
        switch (c) {
            case 'm': mode = optarg; break;
            case 'a': map_file = optarg; break;
            case 'p': prim_file = optarg; break;
            case 'c': mesh_file = optarg; break;
            case 's': start_str = optarg; break;
            case 'g': goal_str = optarg; break;
            case 'o': out_prefix = optarg; break;
            case 'w': w = stof(optarg); break;
            case 'h': print_help(); return 0;
        }
    }

    if (mode == "single") {
        if (map_file.empty() || prim_file.empty() || mesh_file.empty() || 
            start_str.empty() || goal_str.empty()) {
            cerr << "Error: Missing arguments for single mode." << endl;
            print_help();
            return 1;
        }
        HEAP = new MyHEAP();
        solve_single_instance(map_file, prim_file, mesh_file, start_str, goal_str, out_prefix, w);
        delete HEAP;
    }
    else if (mode == "benchmark") {
        // Run the mass testing with fork
        run_fork_benchmark();
    } 
    else {
        print_help();
    }

    return 0;
}
