#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


int main()
{
    auto rviz{rviz::Viz::instance()};

    const rviz::Pose pose{};
    rviz->draw_pose("hello", pose);
    while (!rviz->closed()) {
        rviz->render();
    }

    return 0;
}
