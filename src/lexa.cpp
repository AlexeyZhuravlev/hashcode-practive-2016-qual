#include <common.h>

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <iostream>
#include <cassert>
#include <cmath>
#include <string>
#include <queue>
#include <set>
#include <map>
#include <cstdlib>
#include <chrono>

using namespace std;

struct MySolver : public Context {
    void Solve() {
        Solution.resize(n_drones);
        Solution[0].push_back({Load, 0, 0, 1});
        Solution[0].push_back({Load, 0, 1, 1});
        Solution[0].push_back({Deliver, 0, 0, 1});
        Solution[0].push_back({Load, 1, 2, 1});
        Solution[0].push_back({Deliver, 0, 2, 1});
        Solution[1].push_back({Load, 1, 2, 1});
        Solution[1].push_back({Deliver, 2, 2, 1});
        Solution[1].push_back({Load, 0, 0, 1});
        Solution[1].push_back({Deliver, 1, 0, 1});
    }
};

int main() {
    MySolver solver;

    solver.Input();

    auto start = std::chrono::system_clock::now();
    cerr << "Started solving..." << endl;
    solver.Solve();
    cerr << "Done!" << endl;
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    cerr << "Test time: " << elapsed_seconds.count() << endl;

    cerr << "Outputting" << endl;
    solver.Output();

    cerr << "Scoring" << endl;
    cerr << solver.GetScore() << endl;
    return 0;
}
