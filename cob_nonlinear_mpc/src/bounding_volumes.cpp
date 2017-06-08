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

void BoundingVolume::generate_bounding_volumes(Robot* robot){
    // Get bounding volume forward kinematics
    /*    for(int i=0; i<transform_vec_bvh_.size(); i++)
        {
            T_BVH bvh = transform_vec_bvh_.at(i);
            std::vector<SX> bvh_arm;
            if(i-1<0)
            {
                SX transform = mtimes(fk_vector_.at(i),bvh.T);
                SX tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                bvh_arm.push_back(tmp);
                bvh_matrix[bvh.link].push_back(bvh_arm);

                if(bvh.constraint)
                {
                    bvh_arm.clear();
                    tmp.clear();
                    transform = mtimes(fk_vector_.at(i),bvh.BVH_p);
                    tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                    bvh_arm.push_back(tmp);
                    bvh_matrix[bvh.link].push_back(bvh_arm);
                }
            }
            else
            {
                bvh_arm.clear();
                SX transform = mtimes(fk_vector_.at(i-1),bvh.T);
                SX tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                bvh_arm.push_back(tmp);
                bvh_matrix[bvh.link].push_back(bvh_arm);
                bvh_arm.clear();

                if(bvh.constraint)
                {
                    tmp.clear();
                    transform = mtimes(fk_vector_.at(i-1),bvh.BVH_p);
                    tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                    bvh_arm.push_back(tmp);
                    bvh_matrix[bvh.link].push_back(bvh_arm);
                }
            }
        }
        if(robot_.base_active_)
        {
            for(int i=0; i<bvb_positions_.size(); i++)
            {
                std::vector<SX> base_bvh;
                SX tmp = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+bvb_positions_.at(i)});
                base_bvh.push_back(tmp);
                bvh_matrix["body"].push_back(base_bvh);
            }

        }*/
}
