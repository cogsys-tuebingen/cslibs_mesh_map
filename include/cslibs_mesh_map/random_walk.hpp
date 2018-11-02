#ifndef RANDOM_WALK_HPP
#define RANDOM_WALK_HPP
#include <cslibs_mesh_map/edge_particle.h>
#include <cslibs_mesh_map/mesh_map_tree.h>
namespace cslibs_mesh_map {

struct RandomWalk
{
    RandomWalk(double distance = 0.025):
        generator_(rd_()),
        momentum_(0,distance),
        jump_map_(0.0,1.0)
    {

    }

    void update(EdgeParticle & p, MeshMapTree& map)
    {
        double delta_p = momentum_(generator_);
        MeshMapTree* active = &map.getMap(p.map_id);
        MeshMap* mesh = active_->map_;
        cslibs_math_3d::Vector3d start = p.getActiveVertex(mesh);
        double last_dist = p.getDistanceFromStart();
        if(mesh.isBoundry(p.active_vertex)){

        }
        while(delta_p > 0){
            double d = p.getDistanceToGoal();
            if(d > delta_p){
                p.s += delta_p/p.e;
                break;
            }  else{
                delta_p -= d;
                p.active_vertex = p.goal_vertex;
                p.goal_vertex = mesh.getRandomNeighbourGreaterDistance(p.active_vertex, start, last_dist);
                p.updateEdgeLength(mesh);

                double dist_norm = delta_p /p.e;
                p.s = std::min(dist_norm,1.0);
                delta_p -= p.getDistanceFromStart();
            }
        }
    }

    void jumpMap(EdgeParticle & p, cslibs_math_3d::Vector3d& start, MeshMapTree* current_mesh, MeshMapTree tree)
    {
        if(jump_map_(generator_) > 0.5){
            double dist_from_base = p.getPosition(current_mesh).length();
            double dist_to_child = std::numeric_limits<double>::infinity();
            std::size_t min_id = 0;
            for(std::size_t i = 0; i < current_mesh->children_.size(); ++i){
                MeshMapTree& c = current_mesh->children_[i];
                double d = c.transform.translation().length();
                if(d < dist_to_child){
                    dist_to_child = d;
                    min_id = 0;
                }
            }
            if(dist_from_base < dist_to_child){
                current_mesh = &current_mesh->children_[min_id];
                p.active_vertex = current_mesh->getRandomBoundryVertexFront();
                p.goal_vertex = current_mesh->getRandomNeighbour(p.active_vertex);
            } else{
//                current_mesh = &tree.
                //TODO use parent frame;
            }


        }
    }

    std::vector<EdgeParticle> createParticleSetForOneMap(std::size_t n_particle, MeshMapTree& map)
    {
        const MeshMap& mesh = map.map_;
        double edges_length = mesh.sumEdgeLength();
        std::size_t n_edges = mesh.numbeOfEdges();

        /// prepare ordered sequence of random numbers
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::vector<double> u(n_particle, std::pow(dist(generator_), 1.0 / static_cast<double>(n_particle)));
        {

            for(std::size_t k = n_particle - 1 ; k > 0 ; --k) {
                const double u_ = std::pow(dist(generator_), 1.0 / static_cast<double>(k));
                u[k-1] = u[k] * u_;
            }
        }

        /// draw samples
        std::vector<EdgeParticle> particle_set;
        {
            auto edge_it  = mesh.edgeBegin();
            double cumsum_last = 0.0;
            double cumsum = mesh.calculateEdgeLength(edge_it)/edges_length;

            auto in_range = [&cumsum, &cumsum_last] (double u)
            {
                return u >= cumsum_last && u < cumsum;
            };


            for(auto &u_r : u) {
                while(!in_range(u_r)) {
                    ++edge_it;
                    cumsum_last = cumsum;
                    cumsum += mesh.calculateEdgeLength(edge_it)/edges_length;
                }
                EdgeParticle p;
                p.active_vertex = mesh.fromVertexHandle(edge_it);
                p.goal_vertex = mesh.toVertexHandle(edge_it);
                p.updateEdgeLength(mesh);
                p.s = 0;
                p.map_id = mesh.id_;
                particle_set.push_back(p);
            }
        }

        return particle_set;
    }

    std::random_device rd_;
    std::mt19937 generator_;
    std::uniform_real_distribution<double> momentum_;
    std::uniform_real_distribution<double> jump_map_;
};
}
#endif // RANDOM_WALK_HPP
