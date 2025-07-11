#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <dynamic_gap/gap_detection/GapDetector.h>   // from D-Gap
#include <dynamic_gap/GapPolarArray.h>               // auto-generated
#include <dynamic_gap/GapPolar.h>

using namespace dynamic_gap;

class GapStreamerPolar
{
public:
  GapStreamerPolar(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // ── (1)  load or hard-code the Dynamic-Gap config ────────────────
    // cfg_.loadRosParamFromNodeHandle("gap_streamer_polar");

    // Fill cfg_
    cfg_.map_frame_id     = "map";
    cfg_.odom_frame_id    = "TBD";
    cfg_.robot_frame_id   = "TBD";
    cfg_.sensor_frame_id  = "TBD";
    cfg_.odom_topic       = "TBD";
    cfg_.acc_topic        = "TBD";
    cfg_.scan_topic       = "TBD";
    cfg_.ped_topic        = "/pedsim_simulator/simulated_agents";

    cfg_.rbt.r_inscr      = 0.2f;
    cfg_.rbt.vx_absmax    = 1.0f;
    cfg_.rbt.vy_absmax    = 1.0f;
    cfg_.rbt.vang_absmax  = 1.0f;

    cfg_.scan.angle_min       = -M_PI;
    cfg_.scan.angle_max       =  M_PI;
    cfg_.scan.half_scan       = 256;
    cfg_.scan.half_scan_f     = 256.0f;
    cfg_.scan.full_scan       = 512;
    cfg_.scan.full_scan_f     = 512.0f;
    cfg_.scan.angle_increment = (2 * M_PI) / (cfg_.scan.full_scan_f - 1);
    cfg_.scan.range_min       = 0.03f;
    cfg_.scan.range_max       = -1e10f;

    cfg_.planning.gap_prop                = 1;
    cfg_.planning.pursuit_guidance_method = 1;
    cfg_.planning.holonomic               = false;
    cfg_.planning.future_scan_propagation = true;
    cfg_.planning.egocircle_prop_cheat    = false;
    cfg_.planning.projection_operator     = true;
    cfg_.planning.gap_feasibility_check   = true;
    cfg_.planning.perfect_gap_models      = false;

    cfg_.ctrl.man_ctrl      = false;
    cfg_.ctrl.mpc_ctrl      = false;
    cfg_.ctrl.feedback_ctrl = true;
    cfg_.ctrl.ctrl_ahead_pose = 2;

    cfg_.goal.xy_global_goal_tolerance  = 0.2f;
    cfg_.goal.yaw_global_goal_tolerance = M_PI;
    cfg_.goal.xy_waypoint_tolerance     = 0.1f;

    cfg_.gap_assoc.assoc_thresh = 1.0f;
    cfg_.gap_manip.rgc_angle    = 1.0f;

    cfg_.traj.integrate_maxt        = 10.0f;
    cfg_.traj.integrate_stept       = 0.5f;
    cfg_.traj.max_pose_to_scan_dist = 0.5f;
    cfg_.traj.Q                     = 1.0f;
    cfg_.traj.pen_exp_weight        = 5.0f;
    cfg_.traj.inf_ratio             = 1.5f;
    cfg_.traj.Q_f                   = 1.0f;

    cfg_.projection.k_po_x  = 1.0f;
    cfg_.projection.r_unity = 0.5f;
    cfg_.projection.r_zero  = 1.0f;


    gap_detector_.reset(new GapDetector(cfg_));

    scan_sub_ = nh_.subscribe("scan", 5, &GapStreamerPolar::scanCB, this);
    gap_pub_  = nh_.advertise<dynamic_gap::GapPolarArray>("simplified_gaps", 5);
  }

private:
  // ---------- laser-scan callback ------------------------------------
  void scanCB(const sensor_msgs::LaserScanConstPtr& scan)
  {
    // copy because preprocess mutates the ranges
    auto scan_mut = boost::make_shared<sensor_msgs::LaserScan>(*scan);
    gap_detector_->preprocessScan(scan_mut);

    // gap detection  →  simplification
    auto raw  = gap_detector_->gapDetection(scan_mut);
    auto simp = gap_detector_->gapSimplification(raw);

    // fill the outgoing message (POLAR)
    dynamic_gap::GapPolarArray out;
    out.header = scan->header;

    const float angle_min = scan->angle_min;
    const float inc       = scan->angle_increment;

    for (Gap* g : simp)
    {
      dynamic_gap::GapPolar m;

      // index->angle helpers
      auto idx2ang = [&](int idx){ return angle_min + idx * inc; }; // todo: idk if this is correct. Might cause issues

      m.right_angle = idx2ang(g->RIdx());
      m.right_range = g->RRange();
      m.left_angle  = idx2ang(g->LIdx());
      m.left_range  = g->LRange();

      // width in Euclidean space (optional but handy)
      float rX =  m.right_range * std::cos(m.right_angle);
      float rY =  m.right_range * std::sin(m.right_angle);
      float lX =  m.left_range  * std::cos(m.left_angle );
      float lY =  m.left_range  * std::sin(m.left_angle );
      m.width  = std::hypot(lX - rX, lY - rY);

      out.gaps.push_back(m);
    }

    gap_pub_.publish(out);
    ROS_INFO_STREAM("Published " << out.gaps.size() << " gaps");


    // free memory allocated by GapDetector
    for (Gap* g : raw)  delete g;
    for (Gap* g : simp) delete g;
  }

  // ---------- members ------------------------------------------------- 
  ros::NodeHandle  nh_, pnh_;
  ros::Subscriber   scan_sub_;
  ros::Publisher    gap_pub_;

  dynamic_gap::DynamicGapConfig cfg_;
  std::unique_ptr<GapDetector>       gap_detector_;
};

int main(int argc,char** argv)
{
  ros::init(argc, argv, "gap_streamer_polar");
  ros::NodeHandle nh, pnh("~");
  GapStreamerPolar gsp(nh, pnh);
  ros::spin();
  return 0;
}
