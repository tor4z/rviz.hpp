#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


int main()
{
    auto rviz{rviz::Viz::instance()};

    while (!rviz->closed()) {
        rviz->render();
    }

    return 0;
}
