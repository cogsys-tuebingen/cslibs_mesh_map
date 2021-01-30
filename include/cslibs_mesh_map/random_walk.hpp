#ifndef RANDOM_WALK_HPP
#define RANDOM_WALK_HPP
#include <cslibs_mesh_map/edge_particle.h>
#include <cslibs_mesh_map/mesh_map_tree.h>
namespace cslibs_mesh_map {

struct RandomWalk
{
    RandomWalk(double distance = 0.025, double jump_probability = 0.5):
        generator_(rd_()),
        momentum_(0,distance),
        jump_map_(0.0,1.0),
        jump_probability_(jump_probability)
    {

    }
    inline void updateDistanceToTravel()
    {
        distance_to_travel_ = momentum_(generator_);
    }

    inline void update(EdgeParticle & p, const MeshMapTree& map)
    {
        double delta_p = distance_to_travel_;
        update(p, map, delta_p);
    }

    inline void update(EdgeParticle & p, const MeshMapTree& map, double delta_p)
    {
        const MeshMapTreeNode* active = map.getNode(p.map_id);
//        std::cout << "active address" << active << std::endl;
        if(!active){
            std::cerr << "Map with id " << p.map_id << " not found" <<std::endl;
            return;
        }
        if(active->map.isBoundry(p.active_vertex)){
            randomJumpMap(p, active, map);
        }
        const MeshMap& mesh = active->map;
        if(mesh.id_ != p.map_id){
            std::cerr << "Map id does not fit" <<std::endl;
        }
        cslibs_math_3d::Vector3d start = p.getActiveVertex(mesh);
        double last_dist = p.getDistanceFromStart();
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

    inline void randomJumpMap(EdgeParticle & p, MeshMapTreeNode const* & current_node, const MeshMapTree& tree)
    {
        if(jump_map_(generator_) < jump_probability_){
            return;
        }

        bool front_empty = current_node->map.frontBoundryVertices().empty();
        bool end_empty = current_node->map.backBoundryVertices().empty();
        if(front_empty && end_empty) {
            std::cerr << current_node->frameId() << ": At least one boundry expected; no boundry found!" << std::endl;
            std::cerr << "Disableing jumping to neighbouring map" << std::endl;
            jump_probability_ = 10.0;
            return;
        }

        cslibs_math_3d::Vector3d pos = p.getPosition(current_node->map);
        double dist_from_base = pos.length();
        double dist_to_child = std::numeric_limits<double>::infinity();
        std::size_t min_id = 0;
        for(std::size_t i = 0; i < current_node->children.size(); ++i){
            const MeshMapTreeNode* c = current_node->children[i];
            double d = (c->transform.translation() - pos).length();
            if(d < dist_to_child){
                dist_to_child = d;
                min_id = i;
            }
        }

        if((dist_from_base > dist_to_child && !front_empty) || end_empty){
            const MeshMapTreeNode* ptr = current_node->children[min_id];
            current_node = ptr;
            p.active_vertex = current_node->map.getRandomBoundryVertexFront();
            p.goal_vertex = current_node->map.getRandomNeighbour(p.active_vertex);
            p.s = 0;
            p.map_id = current_node->map.id_;
            p.updateEdgeLength(current_node->map);
        } else{
            if(current_node->parent && !end_empty){
                current_node = current_node->parent;
                p.active_vertex = current_node->map.getRandomBoundryVertexEnd();
                p.goal_vertex = current_node->map.getRandomNeighbour(p.active_vertex);
                p.s = 0;
                p.map_id = current_node->map.id_;
                p.updateEdgeLength(current_node->map);
            }
        }
    }

    inline std::vector<EdgeParticle> createParticleSetForOneMap(std::size_t n_particle, const MeshMapTreeNode& map)
    {
        const MeshMap& mesh = map.map;
        double edges_length = mesh.sumEdgeLength();
        //        std::size_t n_edges = mesh.numberOfEdges();

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
    double distance_to_travel_;
    double jump_probability_;
};
}
#endif // RANDOM_WALK_HPP
