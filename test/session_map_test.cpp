#include <ikd-Tree/ikd_Tree.h>
#include <stdexcept>
#include <memory>

int main()
{
    using Tree = KD_TREE<pcl::PointXYZINormal>;
    for (int session = 0; session < 20; ++session)
    {
        auto tree = std::make_unique<Tree>();
        Tree::PointVector points;
        for (int index = 0; index < 2500; ++index)
        {
            pcl::PointXYZINormal point{};
            point.x = static_cast<float>(index);
            point.y = static_cast<float>(session);
            points.push_back(point);
        }
        tree->Build(points);
        const auto snapshot = tree->Snapshot();
        if (snapshot.size() != points.size())
            throw std::runtime_error("Map snapshot lost points");
        for (const auto& point : snapshot)
            if (point.y != session)
                throw std::runtime_error("Previous session leaked into snapshot");
        // Destructor joins the worker and frees both map and sentinel nodes.
    }
    return 0;
}
