#include <algorithm>
#include <cmath>

#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ",
                      "EXEC_TRAJ", "RETURN_HOME", "RESUME_FROM_RETURN", "FINISH" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  home_set_ = false;
  returning_home_ = false;
  resume_pos_set_ = false;
  resuming_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

  // External return-home trigger:
  // - publish `/planning/return_home` (std_msgs/Empty) to request return planning
  // - this node also subscribes to it, and when received it switches to RETURN_HOME
  return_pub_ = nh.advertise<std_msgs::Empty>("/planning/return_home", 10);
  return_sub_ = nh.subscribe("/planning/return_home", 1, &FastExplorationFSM::returnCallback, this);

  resume_sub_ = nh.subscribe("/planning/resume_explore", 1, &FastExplorationFSM::resumeCallback, this);
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        if (!home_set_) {
          ROS_WARN("No home point set, finish directly.");
          transitState(FINISH, "FSM");
        } else {
          returning_home_ = true;
          resuming_ = false;
          // Save the position before return so that resume can go back here.
          resume_pos_ = fd_->odom_pos_;
          resume_pos_set_ = true;
          fd_->static_state_ = true;
          // Notify other modules (and this node itself) to start return-home planning.
          return_pub_.publish(std_msgs::Empty());
          transitState(RETURN_HOME, "FSM");
        }
        fd_->static_state_ = true;
        clearVisMarker();
      } else if (res == FAIL) {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("plan fail");
        fd_->static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (resuming_) {
        const double dist_to_resume = (fd_->odom_pos_ - resume_pos_).norm();
        const double vel_norm = fd_->odom_vel_.norm();
        if (dist_to_resume < 0.30 && vel_norm < 0.30) {
          resuming_ = false;
          fd_->static_state_ = true;
          transitState(PLAN_TRAJ, "FSM");
          ROS_INFO("Resume point reached. dist=%.3f, vel=%.3f", dist_to_resume, vel_norm);
          return;
        }
        const double return_replan_thresh = std::max(fp_->replan_thresh1_, 0.50);
        if (time_to_end < return_replan_thresh) {
          fd_->static_state_ = false;
          transitState(RESUME_FROM_RETURN, "FSM");
          ROS_WARN("Resume replan: near traj end but not at resume point. dist=%.3f, vel=%.3f",
                   dist_to_resume, vel_norm);
        }
        return;
      }
      if (returning_home_) {
        const double dist_to_home = (fd_->odom_pos_ - home_pos_).norm();
        const double vel_norm = fd_->odom_vel_.norm();
        // Finish return-home only when really close to home and almost stopped.
        if (dist_to_home < 0.30 && vel_norm < 0.30) {
          returning_home_ = false;
          fd_->static_state_ = true;
          transitState(FINISH, "FSM");
          ROS_INFO("Return-home trajectory finished. dist=%.3f, vel=%.3f", dist_to_home, vel_norm);
          return;
        }
        // If current return-home trajectory is near its end but home is still far away,
        // trigger another return-home replanning cycle (always keep returning regardless of distance).
        const double return_replan_thresh = std::max(fp_->replan_thresh1_, 0.50);
        if (time_to_end < return_replan_thresh) {
          fd_->static_state_ = false;
          transitState(RETURN_HOME, "FSM");
          ROS_WARN("Return-home replan: near traj end but home still far. dist=%.3f, vel=%.3f",
                   dist_to_home, vel_norm);
        }
        return;
      }
      if (time_to_end < fp_->replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      if (t_cur > fp_->replan_thresh3_ && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: periodic call=======================================");
      }
      break;
    }

    case RETURN_HOME: {
      LocalTrajData* info = &planner_manager_->local_data_;
      if (fd_->static_state_) {
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      replan_pub_.publish(std_msgs::Empty());
      ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

      // 与 planExploreMotion 一致：远距离用 A* 走廊 + 分段轨迹，避免长距离 kinodynamic 搜索 run out of memory
      const double radius_far = 5.0;
      const double radius_close = 1.5;
      auto yaw_time_lb = [&](double end_yaw_val) -> double {
        if (ViewNode::yd_ <= 1e-6) return -1.0;
        double diff = fabs(end_yaw_val - fd_->start_yaw_(0));
        return std::min(diff, 2 * M_PI - diff) / ViewNode::yd_;
      };

      vector<Vector3d> path_home;
      bool has_astar = expl_manager_->getPathToPosition(fd_->start_pt_, home_pos_, path_home);
      bool plan_success = false;
      Vector3d seg_goal = home_pos_;
      double end_yaw = atan2(home_pos_(1) - fd_->start_pt_(1), home_pos_(0) - fd_->start_pt_(0));

      if (has_astar) {
        double len = Astar::pathLength(path_home);
        if (len < radius_close) {
          double tlb = yaw_time_lb(end_yaw);
          planner_manager_->planExploreTraj(path_home, fd_->start_vel_, fd_->start_acc_, tlb);
          plan_success = true;
        } else if (len > radius_far) {
          vector<Vector3d> truncated_path = { path_home.front() };
          double len2 = 0.0;
          for (size_t i = 1; i < path_home.size() && len2 < radius_far; ++i) {
            len2 += (path_home[i] - truncated_path.back()).norm();
            truncated_path.push_back(path_home[i]);
          }
          seg_goal = truncated_path.back();
          end_yaw = atan2(seg_goal(1) - fd_->start_pt_(1), seg_goal(0) - fd_->start_pt_(0));
          double tlb = yaw_time_lb(end_yaw);
          planner_manager_->planExploreTraj(truncated_path, fd_->start_vel_, fd_->start_acc_, tlb);
          plan_success = true;
        } else {
          seg_goal = home_pos_;
          end_yaw = atan2(seg_goal(1) - fd_->start_pt_(1), seg_goal(0) - fd_->start_pt_(0));
          double tlb = yaw_time_lb(end_yaw);
          plan_success = planner_manager_->kinodynamicReplan(
              fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, seg_goal, Vector3d::Zero(), tlb);
          if (!plan_success) {
            planner_manager_->planExploreTraj(path_home, fd_->start_vel_, fd_->start_acc_, tlb);
            plan_success = true;
          }
        }
      }

      if (!plan_success) {
        Vector3d to_h = home_pos_ - fd_->start_pt_;
        double d = to_h.norm();
        if (d < 0.08) {
          returning_home_ = false;
          fd_->static_state_ = true;
          transitState(FINISH, "FSM");
          break;
        }
        Vector3d inter = fd_->start_pt_ + to_h.normalized() * std::min(d, radius_far);
        end_yaw = atan2(inter(1) - fd_->start_pt_(1), inter(0) - fd_->start_pt_(0));
        double tlb = yaw_time_lb(end_yaw);
        plan_success = planner_manager_->kinodynamicReplan(
            fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, inter, Vector3d::Zero(), tlb);
        if (!plan_success) {
          vector<Vector3d> simple = { fd_->start_pt_, inter };
          planner_manager_->planExploreTraj(simple, fd_->start_vel_, fd_->start_acc_, tlb);
          plan_success = true;
        }
      }

      if (!plan_success) {
        ROS_WARN("Return-home planning failed, retry.");
        fd_->static_state_ = true;
        break;
      }

      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
      planner_manager_->planYawExplore(fd_->start_yaw_, end_yaw, true, expl_manager_->ep_->relax_time_);

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i) {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i) {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i) {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
      fd_->newest_traj_ = bspline;
      transitState(PUB_TRAJ, "FSM");
      break;
    }

    case RESUME_FROM_RETURN: {
      LocalTrajData* info = &planner_manager_->local_data_;
      if (fd_->static_state_) {
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      replan_pub_.publish(std_msgs::Empty());
      ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

      const double radius_far = 5.0;
      const double radius_close = 1.5;
      auto yaw_time_lb = [&](double end_yaw_val) -> double {
        if (ViewNode::yd_ <= 1e-6) return -1.0;
        double diff = fabs(end_yaw_val - fd_->start_yaw_(0));
        return std::min(diff, 2 * M_PI - diff) / ViewNode::yd_;
      };

      vector<Vector3d> path_to_resume;
      bool has_astar = expl_manager_->getPathToPosition(fd_->start_pt_, resume_pos_, path_to_resume);
      bool plan_success = false;
      Vector3d seg_goal = resume_pos_;
      double end_yaw = atan2(resume_pos_(1) - fd_->start_pt_(1), resume_pos_(0) - fd_->start_pt_(0));

      if (has_astar) {
        double len = Astar::pathLength(path_to_resume);
        if (len < radius_close) {
          double tlb = yaw_time_lb(end_yaw);
          planner_manager_->planExploreTraj(path_to_resume, fd_->start_vel_, fd_->start_acc_, tlb);
          plan_success = true;
        } else if (len > radius_far) {
          vector<Vector3d> truncated_path = { path_to_resume.front() };
          double len2 = 0.0;
          for (size_t i = 1; i < path_to_resume.size() && len2 < radius_far; ++i) {
            len2 += (path_to_resume[i] - truncated_path.back()).norm();
            truncated_path.push_back(path_to_resume[i]);
          }
          seg_goal = truncated_path.back();
          end_yaw = atan2(seg_goal(1) - fd_->start_pt_(1), seg_goal(0) - fd_->start_pt_(0));
          double tlb = yaw_time_lb(end_yaw);
          planner_manager_->planExploreTraj(truncated_path, fd_->start_vel_, fd_->start_acc_, tlb);
          plan_success = true;
        } else {
          seg_goal = resume_pos_;
          end_yaw = atan2(seg_goal(1) - fd_->start_pt_(1), seg_goal(0) - fd_->start_pt_(0));
          double tlb = yaw_time_lb(end_yaw);
          plan_success = planner_manager_->kinodynamicReplan(
              fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, seg_goal, Vector3d::Zero(), tlb);
          if (!plan_success) {
            planner_manager_->planExploreTraj(path_to_resume, fd_->start_vel_, fd_->start_acc_, tlb);
            plan_success = true;
          }
        }
      }

      if (!plan_success) {
        Vector3d to_h = resume_pos_ - fd_->start_pt_;
        double d = to_h.norm();
        if (d < 0.08) {
          resuming_ = false;
          fd_->static_state_ = true;
          transitState(PLAN_TRAJ, "FSM");
          break;
        }
        Vector3d inter = fd_->start_pt_ + to_h.normalized() * std::min(d, radius_far);
        end_yaw = atan2(inter(1) - fd_->start_pt_(1), inter(0) - fd_->start_pt_(0));
        double tlb = yaw_time_lb(end_yaw);
        plan_success = planner_manager_->kinodynamicReplan(
            fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, inter, Vector3d::Zero(), tlb);
        if (!plan_success) {
          vector<Vector3d> simple = { fd_->start_pt_, inter };
          planner_manager_->planExploreTraj(simple, fd_->start_vel_, fd_->start_acc_, tlb);
          plan_success = true;
        }
      }

      if (!plan_success) {
        ROS_WARN("Resume-to-point planning failed, retry.");
        fd_->static_state_ = true;
        break;
      }

      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
      planner_manager_->planYawExplore(fd_->start_yaw_, end_yaw, true, expl_manager_->ep_->relax_time_);

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i) {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i) {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i) {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
      fd_->newest_traj_ = bspline;
      transitState(PUB_TRAJ, "FSM");
      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  // Draw local refined viewpoints info
  // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
  //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  // 1),
  //                           "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  // 0, 1),
  //                           "refine_pair", 0, 6);
  // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
  //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
  //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
  //                               ed_ptr->frontiers_.size()),
  //                               "n_points", i, 6);
  // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
  //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // Draw trajectory
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (++delay < 5) return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
  }

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = ros::Time::now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (ros::Time::now() - t1).toSec();
  //   ROS_WARN("Average astar time: %lf", astar_time);
  // }
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  // 返航点与探索规划同一坐标系：在真正开始探索时取当前位姿，避免首帧 odom 未收敛导致 home 落在错误位置
  home_pos_ = fd_->odom_pos_;
  home_set_ = true;
  ROS_INFO("Home (return) set to: %.3f %.3f %.3f", home_pos_(0), home_pos_(1), home_pos_(2));
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::returnCallback(const std_msgs::EmptyConstPtr& msg) {
  (void)msg;
  if (!fd_->have_odom_) {
    ROS_WARN_THROTTLE(1.0, "returnCallback: no odom.");
    return;
  }
  if (state_ == EXPL_STATE::FINISH) return;
  if (state_ == EXPL_STATE::RETURN_HOME) return;

  // Save the "pre-return" position so that we can resume exploration later.
  resume_pos_ = fd_->odom_pos_;
  resume_pos_set_ = true;
  resuming_ = false;

  returning_home_ = true;
  // Always replan return-home from current pose.
  fd_->static_state_ = true;

  if (!home_set_) {
    home_pos_ = fd_->odom_pos_;
    home_set_ = true;
    ROS_WARN("returnCallback: home not set, using current odom as home.");
  }

  ROS_INFO("Received return-home command, switching to RETURN_HOME.");
  transitState(RETURN_HOME, "returnCallback");
}

void FastExplorationFSM::resumeCallback(const std_msgs::EmptyConstPtr& msg) {
  (void)msg;
  if (!fd_->have_odom_) {
    ROS_WARN_THROTTLE(1.0, "resumeCallback: no odom.");
    return;
  }
  if (state_ != EXPL_STATE::FINISH) {
    ROS_WARN_THROTTLE(1.0, "resumeCallback ignored: not in FINISH.");
    return;
  }
  if (!resume_pos_set_) {
    ROS_WARN("resumeCallback ignored: resume position not set.");
    return;
  }

  resuming_ = true;
  returning_home_ = false;
  fd_->static_state_ = true;
  ROS_INFO("Received resume-explore command, switching to RESUME_FROM_RETURN.");
  transitState(RESUME_FROM_RETURN, "resumeCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      transitState(resuming_ ? RESUME_FROM_RETURN : (returning_home_ ? RETURN_HOME : PLAN_TRAJ),
                    "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
       << endl;
}
}  // namespace fast_planner
