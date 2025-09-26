#pragma once

#include<edt_environment.h>
#include<raycast.hpp>
#include<random>

using std::vector;

class TopoIterator
{
    private:
        std::vector<int> path_nums_;
        std::vector<int> cur_index_;
        int combine_num_;
        int cur_num_;

        void increase(int bit_num)
        {
            cur_index_[bit_num] += 1;
            if (cur_index_[bit_num] >= path_nums_[bit_num]) 
            {
                cur_index_[bit_num] = 0;
                increase(bit_num + 1);
            }
        }
    public:
        TopoIterator(std::vector<int> pn)
        {
            path_nums_ = pn;
            cur_index_.resize(path_nums_.size());
            std::fill(cur_index_.begin(), cur_index_.end(), 0);
            cur_num_ = 0;

            combine_num_ = 1;
            for (int i = 0; i < path_nums_.size(); ++i) 
            {
                combine_num_ *= path_nums_[i] > 0 ? path_nums_[i] : 1;
            }
        }
        TopoIterator() {
        }
        ~TopoIterator() {
        }

        bool nextIndex(std::vector<int>& index) {
            index = cur_index_;
            cur_num_ += 1;

            if (cur_num_ == combine_num_) return false;

            // go to next combination
            increase(0);
            return true;
        }
};

class GraphNode
{
    public:
        enum NODE_TYPE { Guard = 1, Connector = 2};
        enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3};
        GraphNode(){}
        GraphNode(Eigen::Vector3d pos,NODE_TYPE type, int id)
        {
            pos_ = pos;
            type_ = type;
            state_ = NEW;
            id_ = id;
        }
        ~GraphNode(){}

        std::vector<std::shared_ptr<GraphNode>> neighbors_;
        Eigen::Vector3d pos_;
        NODE_TYPE type_;
        NODE_STATE state_;
        int id_;
        typedef std::shared_ptr<GraphNode> Ptr;
    private:
};

class TopologyPRM
{
    private:
        EDTEnvironment::Ptr edt_environment_;
        std::random_device rd_;
        std::default_random_engine eng_;
        std::uniform_real_distribution<double> rand_pos_;

        Eigen::Vector3d sample_r_;
        Eigen::Vector3d translation_;
        Eigen::Matrix3d rotation_;

        std::vector<RayCaster> casters_;
        Eigen::Vector3d offset_;

        std::list<GraphNode::Ptr> graph_;
        std::vector<std::vector<Eigen::Vector3d> > raw_paths_;
        std::vector<std::vector<Eigen::Vector3d> > short_paths_;
        std::vector<std::vector<Eigen::Vector3d> > final_paths_;

        std::vector<Eigen::Vector3d> start_pts_, end_pts_;
        // parameter
        double max_sample_time_;
        int max_sample_num_;
        int max_raw_path_, max_raw_path2_;
        int short_cut_num_;
        Eigen::Vector3d sample_inflate_;
        double resolution_;

        double ratio_to_short_;
        int reserve_num_;

        bool parallel_shortcut_;

        void printGraphNode(const GraphNode::Ptr& g);
        void printGraphNodeList(const std::list<GraphNode::Ptr>& graph);
        std::list<GraphNode::Ptr> createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
        std::vector<std::vector<Eigen::Vector3d> > searchPaths();
        void shortcutPaths();
        vector<vector<Eigen::Vector3d>> pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths);
        vector<vector<Eigen::Vector3d>> selectShortPaths(vector<vector<Eigen::Vector3d>>& paths, int step);

        /* ---------- helper ---------- */
        inline Eigen::Vector3d getSample();
        vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt);  // find pairs of visibile guard
        bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2,
                            Eigen::Vector3d pt);  // test redundancy with existing
                                                    // connection between two guard
        bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                        Eigen::Vector3d& pc, int caster_id = 0);
        bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);
        void pruneGraph();

        void depthFirstSearch(vector<GraphNode::Ptr>& vis);

        vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);
        vector<vector<Eigen::Vector3d>> discretizePaths(vector<vector<Eigen::Vector3d>>& path);

        vector<Eigen::Vector3d> discretizePath(vector<Eigen::Vector3d> path);
        void shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num = 1);

        vector<Eigen::Vector3d> discretizePath(const vector<Eigen::Vector3d>& path, int pt_num);
        bool sameTopoPath(const vector<Eigen::Vector3d>& path1, const vector<Eigen::Vector3d>& path2,
                            double thresh);
        Eigen::Vector3d getOrthoPoint(const vector<Eigen::Vector3d>& path);

        int shortestPath(vector<vector<Eigen::Vector3d>>& paths);


    public:
        double clearance_;
        TopologyPRM(){}
        ~TopologyPRM(){}

        void init(std::string package_path);
        void setEnvironment(const EDTEnvironment::Ptr& env);

        void findTopoPath(Eigen::Vector3d start, Eigen::Vector3d end, std::vector<Eigen::Vector3d> start_pts,
                          std::vector<Eigen::Vector3d> end_pts, std::list<GraphNode::Ptr>& graph,
                          std::vector<std::vector<Eigen::Vector3d> >& raw_paths,
                          std::vector<std::vector<Eigen::Vector3d> >& filtered_paths,
                          std::vector<std::vector<Eigen::Vector3d> >& selected_paths);

        double pathLength(const std::vector<Eigen::Vector3d>& path);
        std::vector<Eigen::Vector3d> pathToGuidePts(std::vector<Eigen::Vector3d>& path, int pt_num);
};
