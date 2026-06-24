#include <array>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <queue>
#include <random>
#include <raylib.h>

class AStarAlgorithm
{
  public:
    // every constexpr in a class is static
    // because it is a property of the class itself and not an instance of the
    // class
    // these are evaluated at compile time
    static constexpr size_t WINDOW_WIDTH { 1000 };
    static constexpr size_t NUM_NODES_ROW { 20 };
    static constexpr size_t NODE_PIXEL_SIZE { WINDOW_WIDTH / NUM_NODES_ROW };
    static constexpr size_t NUM_OBSTACLES { NUM_NODES_ROW * 5 };

  private:
    struct Node
    {
        // these can be unsigned, but we use signed int for simplicity,
        // especially for error checking
        // coordinates for the grid
        int x;
        int y;

        float h_cost {}; // heuristic/ estimated cost
        // we make it max because we only change if smaller than current
        float g_cost {
            std::numeric_limits<float>::max()
        }; // how far it is from the start; actual cost

        float f_cost {
            std::numeric_limits<float>::max()
        }; // estimated + actual cost

        bool is_obstacle {};
        bool is_discovered {};
        bool is_expanded {};

        Node* previous {};
    };

    // represents the graph; std::array introduced in c++11
    std::array<std::array<Node, NUM_NODES_ROW>, NUM_NODES_ROW> m_grid;
    // m_ because member variable of class
    // can also do _ after variable name, stick with salar's style
    Node* m_start {};
    Node* m_end {};

    bool m_is_finished {};

    // this function in salar's implementation was called heuristic
    float manhattan_distance(const Node* a, const Node* b)
    {
        return std::abs(a->x - b->x) + std::abs(a->y - b->y);
    }

    Node* get_node(const int x, const int y)
    {
        if (x < 0 || y < 0 || x >= NUM_NODES_ROW || y >= NUM_NODES_ROW)
            return nullptr;
        return &m_grid[x][y];
    }

    std::vector<Node*> get_neighbors(Node* node)
    {
        // Vector2 provided by raylib library
        // std::vector<Vector2> directions {
        //     { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 }
        // };
        // decided to make it an array to make it constexpr
        // also for better memory and speed
        constexpr std::array<Vector2, 4> directions { Vector2 { 0, 1 },
                                                      Vector2 { 1, 0 },
                                                      Vector2 { 0, -1 },
                                                      Vector2 { -1, 0 } };
        std::vector<Node*> neighbors {};
        for (const auto& direction : directions)
        {
            auto* possible_neighbor { get_node(node->x + direction.x,
                                               node->y + direction.y) };
            if (possible_neighbor == nullptr)
                continue;
            neighbors.push_back(possible_neighbor);
        }
        return neighbors;
    }

    struct CompareNodePtr
    {
        // Node* node {}; // we want node pointers so we don't have to make
        // copies
        // the priority queue uses a max heap internally so we reverse it
        // so that we can get the smallest element
        // bool operator<(const CompareNodePtr& other) const
        // {
        //     return node->f_cost > other.node->f_cost;
        // }
        bool operator()(const Node* node1, const Node* node2)
        {
            // tricking the default max heap to behave like a min heap
            return node1->f_cost > node2->f_cost;
        }
    };

    // std::priority_queue introduced in c++98
    // pass in 1 arg for default; 3 because had to pass in CompareNodePtr
    // 1. data type 2. container 3. comparison type
    // priority_queue is compile time so we have to pass in struct as 3rd arg
    // sorted std::vector might be better on a small grid
    // std::priority_queue better in the long run for bigger grids
    // std::priority_queue -> push: O(log n), pop: O(log n)
    // std::vector -> push: O(n), pop: O(1)
    std::priority_queue<Node*, std::vector<Node*>, CompareNodePtr>
        m_discovered_nodes {};

    std::vector<Node*> m_final_path {};

    void expand_node(Node* node) // not recursive, only updates neighbors
    {
        for (auto* neighbor : get_neighbors(node))
        {
            if (neighbor->is_obstacle)
                continue;
            // actual distance from the start increases by 1
            const float new_g_cost { node->g_cost + 1 };
            // if you encounter a neighbor that has already been discovered
            if (new_g_cost >= neighbor->g_cost)
                continue;
            neighbor->g_cost = new_g_cost;
            neighbor->h_cost = manhattan_distance(neighbor, m_end);
            neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
            neighbor->previous = node;
            neighbor->is_discovered = true;
            m_discovered_nodes.push(neighbor);
        }
        node->is_expanded = true;
    }

    void construct_final_path() // this assumes that we are on the end node
    {
        auto* current { m_end };
        // only executes if we reached end node through another node
        while (current->previous)
        {
            m_final_path.push_back(current);
            current = current->previous;
        }
        m_final_path.push_back(current);
    }

  public:
    AStarAlgorithm() // this initializes the grid with start, end, obstacles
    {
        for (int x {}; x < NUM_NODES_ROW; x++)
        {
            for (int y {}; y < NUM_NODES_ROW; y++)
            {
                Node node { x, y };
                m_grid[x][y] = node;
            }
        }
        m_start = &m_grid[0][0];
        m_end = &m_grid[NUM_NODES_ROW - 1][NUM_NODES_ROW - 1];

        m_start->g_cost = 0;
        m_start->h_cost = manhattan_distance(m_start, m_end);
        m_start->f_cost =
            m_start->h_cost; // technically g_cost + h_cost but g_cost = 0
        m_start->is_discovered = true;

        m_discovered_nodes.push(m_start);

        // random_device since c++11
        std::mt19937 generator { std::random_device()() };
        // closed bound
        std::uniform_int_distribution<int> range { 0, NUM_NODES_ROW - 1 };

        // randomly place obstacles on the grid
        for (size_t i {}; i < NUM_OBSTACLES;)
        {
            int x { range(generator) };
            int y { range(generator) };
            Node* node { get_node(x, y) };
            if (node == nullptr || node == m_start || node == m_end ||
                node->is_obstacle)
                continue;
            node->is_obstacle = true;
            i++; // we only want to increment i if the node is set as an
                 // obstacle
        }
    }

    void update()
    { // TODO: leftoff
        if (m_is_finished)
            return;
        if (m_discovered_nodes.empty())
        {
            m_is_finished = true;
            return;
        }
        // same as Node*
        auto current { m_discovered_nodes.top() };
        m_discovered_nodes.pop();
        if (current->is_expanded)
            return;
        current->is_expanded = true;

        if (current == m_end)
        {
            construct_final_path();
            m_is_finished = true;
            return;
        }
        expand_node(current);
    }

    void draw()
    {
        for (int x {}; x < NUM_NODES_ROW; x++)
        {
            for (int y {}; y < NUM_NODES_ROW; y++)
            {
                const auto* node { get_node(x, y) };
                Color color { GRAY };
                if (node->is_obstacle)
                    color = BLACK;
                if (node->is_discovered)
                    color = ORANGE;
                if (node->is_expanded)
                    color = BLUE;
                DrawRectangle(x * NODE_PIXEL_SIZE, y * NODE_PIXEL_SIZE,
                              NODE_PIXEL_SIZE - 1, NODE_PIXEL_SIZE - 1, color);
            }
        }
        // overrides the gray nodes
        for (const auto* node : m_final_path)
        {
            DrawRectangle(node->x * NODE_PIXEL_SIZE, node->y * NODE_PIXEL_SIZE,
                          NODE_PIXEL_SIZE - 1, NODE_PIXEL_SIZE - 1, PURPLE);
        }
        // nothing overrides the colors of start, end
        DrawRectangle(m_start->x * NODE_PIXEL_SIZE,
                      m_start->y * NODE_PIXEL_SIZE, NODE_PIXEL_SIZE - 1,
                      NODE_PIXEL_SIZE - 1, GREEN);
        DrawRectangle(m_end->x * NODE_PIXEL_SIZE, m_end->y * NODE_PIXEL_SIZE,
                      NODE_PIXEL_SIZE - 1, NODE_PIXEL_SIZE - 1, RED);
    }
};

int main()
{
    InitWindow(AStarAlgorithm::WINDOW_WIDTH, AStarAlgorithm::WINDOW_WIDTH,
               "A Star");
    SetTargetFPS(60);

    AStarAlgorithm a_star {};

    while (!WindowShouldClose())
    {
        PollInputEvents(); // best practice
        a_star.update();
        BeginDrawing();
        ClearBackground(BLACK); // best practice
        a_star.draw();
        EndDrawing();
    }
    return 0;
}
