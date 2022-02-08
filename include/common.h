#pragma once

#include <algorithm>
#include <vector>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <cassert>
#include <cmath>
#include <string>
#include <queue>
#include <set>
#include <map>
#include <cstdlib>
#include <unordered_map>

using namespace std;


struct Context {
    struct Point {
        int r;
        int c;
    };

    enum CommandType {
        Deliver = 'D',
        Load = 'L',
        Unload = 'U',
        Wait = 'W'
    };

    struct Command {
        CommandType type;
        int target_id;
        int product_type;
        int num_products;
        int wait_time;
    };

    using TSolution = vector<vector<Command>>;
    TSolution Solution;

    struct Warehouse {
        Point point;
        vector<int> items; // item counts
    };

    struct Order {
        int id;
        Point point;
        int total_items, total_weight;
        unordered_map<int, int> item_counts;
    };

    struct DroneState {
        Point point;
        unordered_map<int, int> item_counts;
        int available_from;
        int total_weight;
        int id;
        bool next_unload = false;

        bool operator<(const DroneState& other) const {
            if (available_from == other.available_from) {
                return next_unload;
            }
            return available_from < other.available_from;
        }
    };

    int n_drones, n_rows, n_cols, max_load, t_simulation;

    int n_products, n_warehouses, n_orders;

    vector<int> product_weights;
    vector<Warehouse> warehouses;
    vector<Order> orders;

    void Input() {
        cin >> n_rows >> n_cols >> n_drones >> t_simulation >> max_load;
        cin >> n_products;
        product_weights.resize(n_products);
        for (int i = 0; i < n_products; ++i) {
            cin >> product_weights[i];
        }
        cin >> n_warehouses;
        warehouses.resize(n_warehouses, {});
        for (auto &w :  warehouses) {
            cin >> w.point.r >> w.point.c;
            w.items.resize(n_products);
            for (int i = 0; i < n_products; ++i) {
                cin >> w.items[i];
            }
        }
        cin >> n_orders;
        orders.resize(n_orders);
        for (int i = 0; i < n_orders; ++i) {
            auto & o = orders[i];
            o.id = i;
            cin >> o.point.r >> o.point.c;
            cin >> o.total_items;
            for (int j = 0; j < o.total_items; ++j) {
                int item;
                cin >> item;
                o.item_counts[item]++;
                o.total_weight += product_weights[item];
            }
        }
    }

    void Output() {
        for (int i = 0; i < n_drones; ++i) {
            for (int j = 0; j < Solution[i].size(); ++j) {
                auto& command = Solution[i][j];
                cout << i << " " << static_cast<char>(command.type) << " ";
                if (command.type == 'W') {
                    cout << command.wait_time;
                } else {
                    cout << command.target_id << " " << command.product_type << " " << command.num_products;
                }
                cout << endl;
            }
        }
    }

    uint64_t GetScore() {
        vector<Warehouse> warehouses_state = warehouses;
        vector<Order> order_state = orders;
        priority_queue<DroneState> drone_state;
        vector<int> next_commands;
        vector<int> order_delivery;
        order_delivery.assign(n_orders, 0);
        next_commands.assign(n_drones, 0);

        for (int i = 0; i < n_drones; ++i) {
            if (Solution[i].size() > 0) {
                drone_state.push(
                    {warehouses[0].point, {}, 0, i, false}
                );
            }
        }


        while (!drone_state.empty()) {
            cerr << "Command" << endl;
            DroneState drone = drone_state.top();
            drone_state.pop();
            auto& command = Solution[drone.id][next_commands[drone.id]];
            switch (command.type) {
                case Deliver:
                {
                    int order_id = command.target_id;
                    Order& order = order_state[order_id];
                    int distance = Distance(drone.point, order.point);
                    order.item_counts[command.product_type] -= command.num_products;
                    if (order.item_counts[command.product_type] < 0) {
                        cerr << "Invalid num items for order " << order_id << endl;
                        return 0;
                    }
                    drone.item_counts[command.product_type] -= command.num_products;
                    if (drone.item_counts[command.product_type] < 0) {
                        cerr << "No items to deliver for drone " << drone.id << endl;
                        return 0;
                    }
                    drone.available_from += distance + 1;
                    order_delivery[order_id] = drone.available_from;
                    break;
                }
                case Load:
                {
                    int warehouse_id = command.target_id;
                    Warehouse& warehouse = warehouses_state[warehouse_id];
                    int distance = Distance(drone.point, warehouse.point);
                    warehouse.items[command.product_type] -= command.num_products;
                    if (warehouse.items[command.product_type] < 0) {
                        cerr << "No items on warehouse " << warehouse_id << endl;
                        return 0;
                    }
                    drone.item_counts[command.product_type] += command.num_products;
                    drone.total_weight += product_weights[command.product_type] * command.num_products;
                    if (drone.total_weight > max_load) {
                        cerr << "Max load exceeded for drone " << drone.id << endl;
                        return 0;
                    }
                    drone.available_from += distance + 1;
                    break;
                }
                case Unload:
                {
                    int warehouse_id = command.target_id;
                    Warehouse& warehouse = warehouses_state[warehouse_id];
                    int distance = Distance(drone.point, warehouse.point);
                    warehouse.items[command.product_type] += command.num_products;
                    drone.item_counts[command.product_type] -= command.num_products;
                    drone.available_from += distance + 1;
                    break;
                }
                case Wait:
                {
                    drone.available_from += command.wait_time;
                    break;
                }
            }
            if (drone.available_from > t_simulation) {
                cerr << "Drone commands out of simulation " << drone.id << endl;
                return 0;
            }
            ++next_commands[drone.id];
            if (next_commands[drone.id] < Solution[drone.id].size()) {
                drone.next_unload = Solution[drone.id][next_commands[drone.id]].type == Unload;
                drone_state.push(drone);
            }
        }
        uint64_t total_score = 0;
        for (int i = 0; i < n_orders; ++i) {
            bool finished = true;
            for (auto [k, v]: order_state[i].item_counts) {
                if (v != 0) {
                    finished = false;
                    break;
                }
            }
            if (finished) {
                int finish_time = order_delivery[i];
                int score = static_cast<int>(ceil((t_simulation - finish_time) / t_simulation * 100));
                total_score += score;
            }
        }
        return total_score;
    }

    int Distance(const Point& a, const Point& b) {
        double distance = sqrt((a.c - b.c) * (a.c - b.c) + (a.r - b.r) * (a.r - b.r));

        return static_cast<int>(ceil(distance));
    }
};
