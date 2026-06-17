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
    static constexpr size_t NUM_NODES_ROW { 20 };
    static constexpr size_t NODE_PIXEL_SIZE { 50 };
    static constexpr size_t NUM_OBSTACLES { NUM_NODES_ROW * 5 };

  private:
    struct Node
    {
        // possibly can be unsigned
        // coordinates
        int x;
        int y;

        float h_cost {};
        // we want the lowest possible, so we initialize to the highest possible
        float g_cost {
            std::numeric_limits<float>::max()
        }; // how far it is from the start

        float f_cost { std::numeric_limits<float>::max() };

        bool is_obstacle {};
        bool is_open {};
        bool is_closed {};

        Node* parent {};
    };

    // represents the graph; std::array introduced in c++11
    std::array<std::array<Node, NUM_NODES_ROW>, NUM_NODES_ROW> m_grid_;
    // m_ because member variable of class, _ after because its private
    Node* m_start_ {};
    Node* m_end_ {};

    bool m_finished_ {};

    float manhattan_distance(const Node* a, const Node* b)
    {
        return std::abs(a->x - b->x) + std::abs(a->y - b->y);
    }

    Node* get_node(const int x, const int y)
    {
        if (x < 0 || y < 0 || x >= NUM_NODES_ROW || y >= NUM_NODES_ROW)
            return nullptr;
        return &m_grid_[x][y]; // whys it not [x][y]
    }

    struct OpenNode // represents a node in the priority queue
    {
        Node* node {};
        // the priority queue uses a max heap internally
        // so we reverse it so that we can get the smallest element
        bool operator<(const OpenNode& other) const
        {
            return node->f_cost > other.node->f_cost;
        }
    };

    // introduced in c++98
    std::priority_queue<OpenNode> m_opened_nodes_ {};

    std::vector<Node*> m_final_path_ {};

    void expand_node(Node* node)
    {
        node->is_closed = true;
        for (auto* neighbor : get_neighbors(node))
        {
            if (neighbor->is_obstacle)
                continue;
            const float new_g_cost { node->g_cost + 1 };
            if (new_g_cost >= neighbor->g_cost)
                continue;
            neighbor->g_cost = new_g_cost;
            neighbor->h_cost = manhattan_distance(neighbor, m_end_);
            neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
            neighbor->is_open = true;
            neighbor->parent = node;
            m_opened_nodes_.push({ neighbor });
        }
    }

    std::vector<Node*> get_neighbors(Node* node)
    {
        std::vector<Vector2> directions {
            { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 }
        };
        std::vector<Node*> output {};
        for (const auto& direction : directions)
        {
            auto* possible_neighbor { get_node(node->x + direction.x,
                                               node->y + direction.y) };
            if (!possible_neighbor)
                continue;
            output.push_back(possible_neighbor);
        }
        return output;
    }

    void reconstruct_path()
    {
        auto* current { m_end_ };
        while (current->parent)
        {
            m_final_path_.push_back(current);
            current = current->parent;
        }
        m_final_path_.push_back(current);
    }

  public:
    AStarAlgorithm()
    {
        for (int y {}; y < NUM_NODES_ROW; y++)
        {
            for (int x {}; x < NUM_NODES_ROW; x++)
            {
                Node node { x, y };
                m_grid_[x][y] = node;
            }
        }
        m_start_ = &m_grid_[0][0];
        m_end_ = &m_grid_[NUM_NODES_ROW - 1][NUM_NODES_ROW - 1];

        m_start_->g_cost = 0;
        m_start_->h_cost = manhattan_distance(m_start_, m_end_);
        m_start_->f_cost = m_start_->h_cost;
        m_start_->is_open = true;

        m_opened_nodes_.push({ m_start_ });

        // random_device since c++11
        std::mt19937 generator { std::random_device()() };
        // closed bound
        std::uniform_int_distribution<int> range { 0, NUM_NODES_ROW - 1 };

        for (size_t i {}; i < NUM_OBSTACLES; i++)
        {
            int x { range(generator) };
            int y { range(generator) };
            Node* node { get_node(x, y) };
            if (!node || node == m_start_ || node == m_end_ ||
                node->is_obstacle)
                continue;
            node->is_obstacle = true;
            i++; // we only want this to execute if the node is set as an
                 // obstacle
        }
    }

    void update()
    {
        if (m_finished_)
            return;
        if (m_opened_nodes_.empty())
        {
            m_finished_ = true;
            return;
        }
        auto current { m_opened_nodes_.top() };
        m_opened_nodes_.pop();
        if (current.node->is_closed)
            return;
        current.node->is_closed = true;

        if (current.node == m_end_)
        {
            reconstruct_path();
            m_finished_ = true;
            return;
        }
        expand_node(current.node);
    }

    void draw()
    {
        for (int y {}; y < NUM_NODES_ROW; y++)
        {

            for (int x {}; x < NUM_NODES_ROW; x++)
            {
                const auto* node { get_node(x, y) };
                Color color { GRAY };
                if (node->is_obstacle)
                    color = BLACK;
                if (node->is_open)
                    color = ORANGE;
                if (node->is_closed)
                    color = BLUE;
                DrawRectangle(x * NODE_PIXEL_SIZE, y * NODE_PIXEL_SIZE,
                              NODE_PIXEL_SIZE, NODE_PIXEL_SIZE, color);
            }
        }

        for (const auto* node : m_final_path_)
        {
            DrawRectangle(node->x * NODE_PIXEL_SIZE, node->y * NODE_PIXEL_SIZE,
                          NODE_PIXEL_SIZE, NODE_PIXEL_SIZE, YELLOW);
        }

        DrawRectangle(m_start_->x * NODE_PIXEL_SIZE,
                      m_start_->y * NODE_PIXEL_SIZE, NODE_PIXEL_SIZE,
                      NODE_PIXEL_SIZE, GREEN);
        DrawRectangle(m_end_->x * NODE_PIXEL_SIZE, m_end_->y * NODE_PIXEL_SIZE,
                      NODE_PIXEL_SIZE, NODE_PIXEL_SIZE, RED);
    }
};

int main()
{
    InitWindow(1000, 1000, "A Star");
    SetTargetFPS(60);

    AStarAlgorithm a_star {};

    while (!WindowShouldClose())
    {
        PollInputEvents();
        a_star.update();
        BeginDrawing();
        // make an actual grid with black lines
        ClearBackground(BLACK);
        a_star.draw();
        EndDrawing();
    }
    return 0;
}
