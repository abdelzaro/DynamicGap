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
    cfg_.loadRosParamFromNodeHandle("gap_streamer_polar");
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
    auto raw  = gap_detector_->gapDetection(scan_mut, geometry_msgs::PoseStamped());
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

    // free memory allocated by GapDetector
    for (Gap* g : raw)  delete g;
    for (Gap* g : simp) delete g;
  }

  // ---------- members -------------------------------------------------
  ros::NodeHandle  nh_, pnh_;
  ros::Subscriber   scan_sub_;
  ros::Publisher    gap_pub_;

  Config                             cfg_;
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
