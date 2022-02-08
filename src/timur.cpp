a#include <common.h>

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
    vector<Order> current_orders;
    vector<Warehouse> current_warehouses;
    vector<DroneState> drones;
    int current_global_time;

    vector<vector<tuple<int, int, int>>> potential_load_tasks_pool;

    int find_nearest_drone_with_item(const Point& p, int item) {
        int min_dist, mini = -1;
        for (int i = 0; i < drones.size(); ++i) {
            auto& d = drones[i];
            if (d.item_counts.count(item)) {
                int dist = Distance(p, d.point) + d.available_from - current_global_time;
                if (mini == -1 or dist < min_dist) {
                    min_dist = dist;
                    mini = i;
                }
            }
        }
        return mini;
    }

    int find_nearest_warehouse_with_item(const Point& p, int item) {
        int min_dist, mini = -1;
        for (int i = 0; i < current_warehouses.size(); ++i) {
            auto& w = current_warehouses[i];
            if (w.items[item] > 0) {
                int dist = Distance(p, w.point);
                if (mini == -1 or dist < min_dist) {
                    min_dist = dist;
                    mini = i;
                }
            }
        }
        return mini;
    }

    vector<int> rank_orders() {
        vector<double> perdolnoct_ordera;
        potential_load_tasks_pool.assign(orders.size(), {});

        vector<int> ranking;
        ranking.resize(n_orders);
        for (int i = 0; i < n_orders; ++i) {
            ranking[i] = i;
        }

        const double WAREHOUSE_COEF = 5.;
        const double MAX_PERDOLNOST = 1e9;
        for (int order_i = 0; order_i < orders.size(); ++order_i) {
            auto& order = current_orders[order_i];
            bool is_completed = true;
            double cur_perdolnost = 0;
            for (const auto [item, cnt]: order.item_counts) {
                if (cnt > 0) {
                    is_completed = false;
                }
                int cur_cnt = cnt;
                while (cur_cnt > 0) {
                    int d_id = find_nearest_drone_with_item(order.point, item);
                    int w_id = find_nearest_warehouse_with_item(order.point, item);
                    int warehouse_dist = Distance(current_warehouses[w_id].point, order.point);
                    if (d_id != -1) {
                        int drone_dist = Distance(drones[d_id].point, order.point);
                        if (w_id == -1 || drone_dist < warehouse_dist * WAREHOUSE_COEF) {  // heuristic
                            cur_perdolnost += drone_dist;
                            cur_cnt -= drones[d_id].item_counts[item];
                            continue;
                        }
                    }
                    // else
                    cur_perdolnost += warehouse_dist * WAREHOUSE_COEF;  // heuristic
                    potential_load_tasks_pool[order_i].emplace_back(w_id, item,
                                                                    min(current_warehouses[w_id].items[item], cur_cnt));
                    cur_cnt -= current_warehouses[w_id].items[item];
                }
            }
            if (is_completed) {
                order.item_counts.clear();
            }
            perdolnoct_ordera.push_back(!is_completed ? cur_perdolnost : MAX_PERDOLNOST);
        }
        sort(ranking.begin(), ranking.end(), [&](int a, int b) {
            return perdolnoct_ordera[a] < perdolnoct_ordera[b];
        });
        return ranking;
    }

    multiset<tuple<int, int, int>> load_tasks_pool;
    set<tuple<int, int, int>> deliver_pool;

    void update_tasks_pool() {
        auto ranking = rank_orders();
        const int MAX_CURRENT_ORDERS = 100; // heuristic

        load_tasks_pool.clear();
        deliver_pool.clear();

        for (int i = 0; i < min(MAX_CURRENT_ORDERS, (int)ranking.size()); ++i) {
            int order_id = ranking[i];
            load_tasks_pool.insert(potential_load_tasks_pool[order_id].begin(), potential_load_tasks_pool[order_id].end());
            for (const auto [item, cnt]: current_orders[order_id].item_counts) {
                deliver_pool.emplace(order_id, item, cnt);
            }

        }
    }

    void do_load(multiset<tuple<int, int, int>>::iterator it, int drone_id) {
        auto [w_id, item, cnt] = *it;
        int max_items = min({cnt, current_warehouses[w_id].items[item],
                            (max_load - drones[drone_id].total_weight) / product_weights[item]});
        if (max_items <= 0) {
            cerr << "do load " << current_global_time << " " << cnt << " " << current_warehouses[w_id].items[item]
                 << " " << (max_load - drones[drone_id].total_weight) << " " << product_weights[item] << endl;
        }
        assert (max_items > 0);
        int dist = Distance(drones[drone_id].point, current_warehouses[w_id].point);
        if (drones[drone_id].available_from + dist + 1 < t_simulation) {
            Solution[drone_id].push_back({Load, w_id, item, max_items});
        }
        drones[drone_id].point = current_warehouses[w_id].point;
        drones[drone_id].item_counts[item] += max_items;
        drones[drone_id].total_weight += max_items * product_weights[item];
        drones[drone_id].available_from += dist + 1;
        current_warehouses[w_id].items[item] -= max_items;

        load_tasks_pool.erase(it);
        if (max_items != cnt) {
            load_tasks_pool.emplace(w_id, item, cnt - max_items);
        }
    }

    inline void decrease_or_delete(unordered_map<int, int>& item_counts, int item, int cnt) {
        auto it = item_counts.find(item);
        it->second -= cnt;
        if (it->second == 0) {
            item_counts.erase(it);
        }
    }

    void do_deliver(set<tuple<int, int, int>>::iterator it, int drone_id) {
        auto [order_id, item, cnt] = *it;
        int max_items = min({cnt, current_orders[order_id].item_counts[item],
                            drones[drone_id].item_counts[item]});
        if (max_items <= 0) {
            cerr << "do deliver " << current_global_time << " " << cnt << " " << current_orders[order_id].item_counts[item]
            << " " << drones[drone_id].item_counts[item] << endl;
        }
        assert (max_items > 0);
        int dist = Distance(drones[drone_id].point, current_orders[order_id].point);
        if (drones[drone_id].available_from + dist + 1 < t_simulation) {
            Solution[drone_id].push_back({Deliver, order_id, item, max_items});
        }
        drones[drone_id].point = current_orders[order_id].point;
        decrease_or_delete(drones[drone_id].item_counts, item, max_items);
        drones[drone_id].total_weight -= max_items * product_weights[item];
        drones[drone_id].available_from += dist + 1;

        decrease_or_delete(current_orders[order_id].item_counts, item, max_items);

        deliver_pool.erase(it);
        if (max_items != cnt) {
            deliver_pool.emplace(order_id, item, cnt - max_items);
        }
    }

    bool make_drone_busy(int drone_id) {
        const int MAX_DIST = 1000000;
        auto best_load_task_it = load_tasks_pool.end();
        int best_load_dist = MAX_DIST;
        for (auto it = load_tasks_pool.begin(); it != load_tasks_pool.end(); ++it) {
            auto [w_id, item, cnt] = *it;
            if (current_warehouses[w_id].items[item] <= 0 ||
                product_weights[item] > max_load - drones[drone_id].total_weight) {
                continue;
            }
            int dist = Distance(current_warehouses[w_id].point, drones[drone_id].point);
            if (best_load_task_it == load_tasks_pool.end() ||
                dist < best_load_dist) {
                best_load_dist = dist;
                best_load_task_it = it;
            }
        }

        auto best_deliver_task_it = deliver_pool.end();
        int best_deliver_dist = MAX_DIST;
        for (auto it = deliver_pool.begin(); it != deliver_pool.end(); ++it) {
            auto [order_id, item, cnt] = *it;
            if (!drones[drone_id].item_counts.count(item)) {
                continue;
            }
            int dist = Distance(current_orders[order_id].point, drones[drone_id].point);
            if (best_deliver_task_it == deliver_pool.end() ||
                dist < best_deliver_dist) {
                best_deliver_dist = dist;
                best_deliver_task_it = it;
            }
        }
        if (best_load_dist == MAX_DIST && best_deliver_dist == MAX_DIST) {
            return false;
        }
        // heuristic
        if (best_deliver_dist == MAX_DIST) {
            do_load(best_load_task_it, drone_id);
        } else if (best_deliver_dist == 0 || best_load_dist == MAX_DIST) {
            do_deliver(best_deliver_task_it, drone_id);
        } else if (best_deliver_dist * 0.7 < best_load_dist) {
            do_deliver(best_deliver_task_it, drone_id);
        } else if (current_global_time + best_load_dist > t_simulation) {
            do_deliver(best_deliver_task_it, drone_id);
        } else {
            do_load(best_load_task_it, drone_id);
        }

        return true;
    }


    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> drones_pool; // available_from, drone_id

    void Solve() {
        current_orders = orders; // to have current state;
        current_warehouses = warehouses;
        Solution.resize(n_drones);

        for (int i =0; i < n_drones; ++i) { // init
            DroneState st;
            st.point = current_warehouses[0].point;
            st.available_from = 0;
            st.total_weight = 0;
            drones.push_back(st);
            drones_pool.emplace(0, i);
        }

        current_global_time = 0;
        const int UPDATE_POOL_EVERY = 10;  // heuristic
        update_tasks_pool();
        int next_update = UPDATE_POOL_EVERY;

        while (!drones_pool.empty()) {
            auto [cur_time, d_id] = drones_pool.top();
            drones_pool.pop();
            current_global_time = cur_time;
            if (cur_time >= t_simulation) {
                break;
            }

            if (cur_time >= next_update) {
                update_tasks_pool();
                next_update += UPDATE_POOL_EVERY;
            }
            bool is_busy = make_drone_busy(d_id);
            if (is_busy)
            {
                drones_pool.push({drones[d_id].available_from, d_id});
            }
        }

        // Solution goes here
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

    cerr << solver.GetScore() << endl;
    return 0;
}
