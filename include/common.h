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
    enum CommandType {
        Deliver = 'D',
        Load = 'L',
        Undload = 'U',
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
        int r, c;
        vector<int> items;
    };

    struct Order {
        int id;
        int r, c;
        int total_items, total_weight;
        unordered_map<int, int> item_counts;
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
            cin >> w.r >> w.c;
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
            cin >> o.r >> o.c;
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
        return 0;
    }
    
};
