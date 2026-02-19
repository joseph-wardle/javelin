import std;

import javelin.core.types;
import javelin.core.app;

using namespace javelin;

int main(int argc, char **argv) {
    std::filesystem::path scene_path = "assets/scenes/samples/stack_boxes.jvscene";
    if (argc > 1 && argv[1] != nullptr) {
        scene_path = argv[1];
    }

    App app{};
    app.run(scene_path);

    return 0;
}
