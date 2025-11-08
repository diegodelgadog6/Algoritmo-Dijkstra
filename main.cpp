#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;

const int INF = numeric_limits<int>::max();

class Graph {
private:
    int V;  // número de nodos
    vector<char> names;  // ['A','B',...]
    vector<vector<pair<int,int>>> adj; // adj[u] = { (v, peso), ... }

    // estos se llenan cuando se ejecuta dijkstra()
    vector<int> last_dist;
    vector<int> last_parent;

    int idx(char name) const {
        for (int i = 0; i < V; i++)
            if (names[i] == name)
                return i;
        return -1;
    }

public:
    Graph(const vector<char>& node_names) {
        V = (int)node_names.size();
        names = node_names;
        adj.assign(V, {});
        last_dist.assign(V, INF);
        last_parent.assign(V, -1);
    }

    // agrega conexión no dirigida u <-> v
    void add_edge(char u_name, char v_name, int weight) {
        int u = idx(u_name);
        int v = idx(v_name);
        if (u == -1 || v == -1) return;
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight});
    }

    // muestra el grafo como lista de adyacencia
    void print_graph() const {
        cout << "Grafo (lista de adyacencia):\n";
        for (int u = 0; u < V; u++) {
            cout << names[u] << ": ";
            for (auto &edge : adj[u]) {
                int v = edge.first;
                int w = edge.second;
                cout << "(" << names[v] << ", " << w << "m) ";
            }
            cout << "\n";
        }
        cout << "\n";
    }

    // algoritmo de Dijkstra usando min-heap
    void dijkstra(char start_name) {
        int start = idx(start_name);
        if (start == -1) return;

        last_dist.assign(V, INF);
        last_parent.assign(V, -1);

        // min-heap: (dist, nodo)
        priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;

        last_dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            if (d > last_dist[u]) continue; // ya hay una mejor

            for (auto &edge : adj[u]) {
                int v = edge.first;
                int w = edge.second;
                if (last_dist[u] + w < last_dist[v]) {
                    last_dist[v] = last_dist[u] + w;
                    last_parent[v] = u;
                    pq.push({last_dist[v], v});
                }
            }
        }
    }

    // imprime el camino óptimo hasta dest_name usando los padres calculados
    void print_shortest_path(char dest_name) const {
        int dest = -1;
        for (int i = 0; i < V; i++)
            if (names[i] == dest_name)
                dest = i;
        if (dest == -1) return;

        if (last_dist[dest] == INF) {
            cout << "No hay camino hacia " << dest_name << ".\n";
            return;
        }

        vector<char> path;
        for (int cur = dest; cur != -1; cur = last_parent[cur]) {
            path.push_back(names[cur]);
        }
        reverse(path.begin(), path.end());

        cout << "Camino mas corto hacia " << dest_name << ": ";
        for (size_t i = 0; i < path.size(); i++) {
            cout << path[i];
            if (i + 1 < path.size()) cout << " -> ";
        }
        cout << " (distancia total: " << last_dist[dest] << " m)\n";
    }

    // imprime distancias a todos después de dijkstra()
    void print_all_distances(char start_name) const {
        cout << "Distancias minimas desde " << start_name << ":\n";
        for (int i = 0; i < V; i++) {
            cout << "  " << start_name << " -> " << names[i] << ": ";
            if (last_dist[i] == INF) cout << "INF\n";
            else cout << last_dist[i] << " m\n";
        }
        cout << "\n";
    }
};

int main() {
    // 1. creamos el grafo con tus 7 nodos
    vector<char> nodes = {'A','B','C','D','E','F','G'};
    Graph g(nodes);

    // 2. agregamos exactamente las aristas que mencionaste
    g.add_edge('A','B',170);
    g.add_edge('A','C',200);
    g.add_edge('A','G',60);
    g.add_edge('A','F',260);

    g.add_edge('B','C',31);

    g.add_edge('C','D',88);

    g.add_edge('D','E',75);
    g.add_edge('D','F',190);

    g.add_edge('E','G',220);

    g.add_edge('F','G',205);

    // 3. mostramos el grafo
    g.print_graph();

    // 4. pedimos nodo origen
    char start;
    cout << "Selecciona un nodo origen (A, B, C, D, E, F o G): ";
    cin >> start;

    g.dijkstra(start);
    g.print_all_distances(start);

    // 5. mostramos al menos 3 caminos optimos (lo hacemos para todos)
    cout << "Caminos optimos desde " << start << ":\n";
    for (char name : nodes) {
        g.print_shortest_path(name);
    }

    return 0;
}
