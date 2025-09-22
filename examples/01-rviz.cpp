#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


int main()
{
    auto rviz{rviz::Viz::instance()};

    const rviz::Pose base{};
    rviz->draw_pose("base", base);

    rviz::Pose cam{};
    cam.x = 5.0f;
    cam.pitch = 3.14f;
    rviz->draw_pose("cam", cam);


    while (!rviz->closed()) {
        rviz->render();
    }

    return 0;
}
