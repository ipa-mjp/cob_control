#include <cob_nonlinear_mpc/bounding_volumes.h>

void BoundingVolume::visualizeBVH(const geometry_msgs::Point point, double radius, int id)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "preview";
    marker.header.frame_id = "odom_combined";


    marker.scale.x = 2*radius;
    marker.scale.y = 2*radius;
    marker.scale.z = 2*radius;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1;

    marker_array_.markers.clear();

    marker.id = id;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker_array_.markers.push_back(marker);

    marker_pub_.publish(marker_array_);
}

void BoundingVolume::generate_bounding_volumes(Robot &robot, ForwardKinematics &fk)
{
    std::vector<SX> fk_vector = fk.getFkVector();
    SX fk_base = fk.getFkBase();

    // Get bounding volume forward kinematics
        for(int i=0; i<fk_vector.size(); i++)
        {
            T_BVH bvh = fk_vector.at(i);
            std::vector<SX> bvh_arm;
            if(i-1<0)
            {
                SX transform = mtimes(fk_vector.at(i),bvh.T);
                SX tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                bvh_arm.push_back(tmp);
                bv_mat[bvh.link].push_back(bvh_arm);

                if(bvh.constraint)
                {
                    bvh_arm.clear();
                    tmp.clear();
                    transform = mtimes(fk_vector.at(i),bvh.BVH_p);
                    tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                    bvh_arm.push_back(tmp);
                    bv_mat[bvh.link].push_back(bvh_arm);
                }
            }
            else
            {
                bvh_arm.clear();
                SX transform = mtimes(fk_vector.at(i-1),bvh.T);
                SX tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                bvh_arm.push_back(tmp);
                bv_mat[bvh.link].push_back(bvh_arm);
                bvh_arm.clear();

                if(bvh.constraint)
                {
                    tmp.clear();
                    transform = mtimes(fk_vector.at(i-1),bvh.BVH_p);
                    tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                    bvh_arm.push_back(tmp);
                    bv_mat[bvh.link].push_back(bvh_arm);
                }
            }
        }
        if(robot.base_active_)
        {
            for(int i=0; i<bvb_positions_.size(); i++)
            {
                std::vector<SX> base_bvh;
                SX tmp = SX::vertcat({fk_base(0,3), fk_base(1,3), fk_base(2,3)+bvb_positions_.at(i)});
                base_bvh.push_back(tmp);
                bv_mat["body"].push_back(base_bvh);
            }
        }
}

SX BoundingVolume::getOutputConstraints(Robot &robot)
{
    std::unordered_map<std::string, std::vector<std::string> >::iterator it_scm;
    SX dist;
    SX barrier;
    int counter = 0;
    double bv_radius;

    for( it_scm = robot.self_collision_map_.begin(); it_scm != robot.self_collision_map_.end(); it_scm++)
    {
        std::vector<string> scm_collision_links = it_scm->second;
        for(int i=0; i<scm_collision_links.size(); i++)
        {
            ROS_WARN_STREAM(it_scm->first);
            vector<vector<SX>> p1_mat = bv_mat[it_scm->first];
            vector<vector<SX>> p2_mat = bv_mat[scm_collision_links.at(i)];

            for(int k=0; k<p1_mat.size(); k++)
            {
                if(it_scm->first == "body")
                {
                    bv_radius = bvb_radius_.at(k);
                }
                else
                {
                    bv_radius = 0.1;
                }

                vector<SX> p1_vec = p1_mat.at(k);
                for(int m=0; m<p2_mat.size(); m++)
                {
                    vector<SX> p2_vec = p2_mat.at(m);

                    SX p1 = SX::vertcat({p1_vec.at(0)});
                    SX p2 = SX::vertcat({p2_vec.at(0)});
                    dist = dot(p1 - p2, p1 - p2);

                    if(counter == 0)
                    {
                        barrier = exp((bv_radius - sqrt(dist))/0.01);
                        counter = 1;
                    }
                    else
                    {
                        barrier += exp((bv_radius - sqrt(dist))/0.01);
                    }
                }
            }
        }
    }
    return barrier;
}


bool BoundingVolume::setBVBpositions(vector<double> bvb_pos)
{
    bvb_positions_ = bvb_pos;
    return true;
}

bool BoundingVolume::setBVBradius(vector<double> bvb_rad)
{
    bvb_radius_ = bvb_rad;
    return true;
}
