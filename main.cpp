#include <raylib.h>

class AStarAlgorithm {
    public:

        void update() {

        }

        void draw() {

        }
};

int main() {
    InitWindow(1000, 1000, "A Star");
    SetTargetFPS(60);
    while (!WindowShouldClose()) {
        PollInputEvents();
        BeginDrawing();
        ClearBackground(BLACK);
        EndDrawing();
    }
    return 0;
}
