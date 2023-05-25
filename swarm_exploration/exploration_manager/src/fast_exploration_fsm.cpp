
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/HGrid.h>
#include <exploration_manager/GridTour.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>
// #include <active_perception/uniform_grid.h>
// #include <lkh_tsp_solver/lkh_interface.h>
// #include <lkh_mtsp_solver/lkh3_interface.h>

#include <fstream>

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
  nh.param("fsm/attempt_interval", fp_->attempt_interval_, 0.2);
  nh.param("fsm/pair_opt_interval", fp_->pair_opt_interval_, 1.0);
  nh.param("fsm/repeat_send_num", fp_->repeat_send_num_, 10);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH", "IDL"
                                                                                              "E" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->avoid_collision_ = false;
  fd_->go_back_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

  // Swarm, timer, pub and sub
  drone_state_timer_ =
      nh.createTimer(ros::Duration(0.04), &FastExplorationFSM::droneStateTimerCallback, this);
  drone_state_pub_ =
      nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_send", 10);
  drone_state_sub_ = nh.subscribe(
      "/swarm_expl/drone_state_recv", 10, &FastExplorationFSM::droneStateMsgCallback, this);

  opt_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::optTimerCallback, this);
  opt_pub_ = nh.advertise<exploration_manager::PairOpt>("/swarm_expl/pair_opt_send", 10);
  opt_sub_ = nh.subscribe("/swarm_expl/pair_opt_recv", 100, &FastExplorationFSM::optMsgCallback,
      this, ros::TransportHints().tcpNoDelay());

  opt_res_pub_ =
      nh.advertise<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res_send", 10);
  opt_res_sub_ = nh.subscribe("/swarm_expl/pair_opt_res_recv", 100,
      &FastExplorationFSM::optResMsgCallback, this, ros::TransportHints().tcpNoDelay());

  swarm_traj_pub_ = nh.advertise<bspline::Bspline>("/planning/swarm_traj_send", 100);
  swarm_traj_sub_ =
      nh.subscribe("/planning/swarm_traj_recv", 100, &FastExplorationFSM::swarmTrajCallback, this);
  swarm_traj_timer_ =
      nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::swarmTrajTimerCallback, this);

  hgrid_pub_ = nh.advertise<exploration_manager::HGrid>("/swarm_expl/hgrid_send", 10);
  grid_tour_pub_ = nh.advertise<exploration_manager::GridTour>("/swarm_expl/grid_tour_send", 10);
}

int FastExplorationFSM::getId() {
  return expl_manager_->ep_->drone_id_;
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(
      1.0, "[FSM]: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom");
        return;
      }
      if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0, "wait for init");
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

    case IDLE: {
      double check_interval = (ros::Time::now() - fd_->last_check_frontier_time_).toSec();
      if (check_interval > 100.0) {
        // if (!expl_manager_->updateFrontierStruct(fd_->odom_pos_)) {
        ROS_WARN("Go back to (0,0,1)");
        // if (getId() == 1) {
        //   expl_manager_->ed_->next_pos_ = Eigen::Vector3d(-3, 1.9, 1);
        // } else
        // expl_manager_->ed_->next_pos_ = Eigen::Vector3d(0, 0.0, 1);
        // Eigen::Vector3d dir = (fd_->start_pos_ - fd_->odom_pos_);
        // expl_manager_->ed_->next_yaw_ = atan2(dir[1], dir[0]);

        expl_manager_->ed_->next_pos_ = fd_->start_pos_;
        expl_manager_->ed_->next_yaw_ = 0.0;

        fd_->go_back_ = true;
        transitState(PLAN_TRAJ, "FSM");
        // } else {
        //   fd_->last_check_frontier_time_ = ros::Time::now();
        // }
      }
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
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
      } else if (res == FAIL) {  // Keep trying to replan
        fd_->static_state_ = true;
        ROS_WARN("Plan fail");
      } else if (res == NO_GRID) {
        fd_->static_state_ = true;
        fd_->last_check_frontier_time_ = ros::Time::now();
        ROS_WARN("No grid");
        transitState(IDLE, "FSM");
        visualize(1);
        // clearVisMarker();
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;

        // fd_->newest_traj_.drone_id = planner_manager_->swarm_traj_data_.drone_id_;
        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_.publish(fd_->newest_traj_);

        thread vis_thread(&FastExplorationFSM::visualize, this, 2);
        vis_thread.detach();
        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      auto tn = ros::Time::now();
      // Check whether replan is needed
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).toSec();

      if (!fd_->go_back_) {
        bool need_replan = false;
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
          ROS_WARN("Replan: cluster covered=====================================");
          need_replan = true;
        } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan if traj is almost fully executed
          ROS_WARN("Replan: traj fully executed=================================");
          need_replan = true;
        } else if (t_cur > fp_->replan_thresh3_) {
          // Replan after some time
          ROS_WARN("Replan: periodic call=======================================");
          need_replan = true;
        }

        if (need_replan) {
          if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
            // Update frontier and plan new motion
            thread vis_thread(&FastExplorationFSM::visualize, this, 1);
            vis_thread.detach();
            transitState(PLAN_TRAJ, "FSM");
          } else {
            // No frontier detected, finish exploration
            fd_->last_check_frontier_time_ = ros::Time::now();
            transitState(IDLE, "FSM");
            ROS_WARN("Idle since no frontier is detected");
            fd_->static_state_ = true;
            replan_pub_.publish(std_msgs::Empty());
            // clearVisMarker();
            visualize(1);
          }
        }
      } else {
        // Check if reach goal
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_.publish(std_msgs::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan for going back
          replan_pub_.publish(std_msgs::Empty());
          transitState(PLAN_TRAJ, "FSM");
          thread vis_thread(&FastExplorationFSM::visualize, this, 1);
          vis_thread.detach();
        }
      }

      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res;
  if (fd_->avoid_collision_ || fd_->go_back_) {  // Only replan trajectory
    res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
    fd_->avoid_collision_ = false;
  } else {  // Do full planning normally
    res = expl_manager_->planExploreMotion(
        fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  }

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

void FastExplorationFSM::visualize(int content) {
  // content 1: frontier; 2 paths & trajs
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
    double a = (drone_id - 1) / double(num + 1);
    double b = 1 / double(num + 1);
    return a + b * double(id) / ed_ptr->frontiers_.size();
  };

  if (content == 1) {
    // Draw frontier
    static int last_ftr_num = 0;
    static int last_dftr_num = 0;
    for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4), "frontier", i, 4);

      // getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
      // double(i) / ed_ptr->frontiers_.size()

      // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first,
      // ed_ptr->frontier_boxes_[i].second,
      //     Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();

    // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
    //   visualization_->drawCubes(
    //       ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
    // for (int i = ed_ptr->dead_frontiers_.size(); i < last_dftr_num; ++i)
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
    // last_dftr_num = ed_ptr->dead_frontiers_.size();

    // // Draw updated box
    // Vector3d bmin, bmax;
    // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax, false);
    // visualization_->drawBox(
    //     (bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0, 4);

    // vector<Eigen::Vector3d> bmins, bmaxs;
    // planner_manager_->edt_environment_->sdf_map_->mm_->getChunkBoxes(bmins, bmaxs, false);
    // for (int i = 0; i < bmins.size(); ++i) {
    //   visualization_->drawBox((bmins[i] + bmaxs[i]) / 2.0, bmaxs[i] - bmins[i],
    //       Vector4d(0, 1, 1, 0.3), "updated_box", i + 1, 4);
    // }

  } else if (content == 2) {

    // Hierarchical grid and global tour --------------------------------
    // vector<Eigen::Vector3d> pts1, pts2;
    // expl_manager_->uniform_grid_->getPath(pts1, pts2);
    // visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0.3, 0, 1), "partition", 0,
    // 6);

    if (expl_manager_->ep_->drone_id_ == 1) {
      vector<Eigen::Vector3d> pts1, pts2;
      expl_manager_->hgrid_->getGridMarker(pts1, pts2);
      visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);

      vector<Eigen::Vector3d> pts;
      vector<string> texts;
      expl_manager_->hgrid_->getGridMarker2(pts, texts);
      static int last_text_num = 0;
      for (int i = 0; i < pts.size(); ++i) {
        visualization_->drawText(pts[i], texts[i], 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      for (int i = pts.size(); i < last_text_num; ++i) {
        visualization_->drawText(
            Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      last_text_num = pts.size();

      // // Pub hgrid to ground node
      // exploration_manager::HGrid hgrid;
      // hgrid.stamp = ros::Time::now().toSec();
      // for (int i = 0; i < pts1.size(); ++i) {
      //   geometry_msgs::Point pt1, pt2;
      //   pt1.x = pts1[i][0];
      //   pt1.y = pts1[i][1];
      //   pt1.z = pts1[i][2];
      //   hgrid.points1.push_back(pt1);
      //   pt2.x = pts2[i][0];
      //   pt2.y = pts2[i][1];
      //   pt2.z = pts2[i][2];
      //   hgrid.points2.push_back(pt2);
      // }
      // hgrid_pub_.publish(hgrid);
    }

    auto grid_tour = expl_manager_->ed_->grid_tour_;
    // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // for (auto& pt : grid_tour) pt = pt + trans;

    visualization_->drawLines(grid_tour, 0.05,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        "grid_tour", 0, 6);

    // Publish grid tour to ground node
    exploration_manager::GridTour tour;
    for (int i = 0; i < grid_tour.size(); ++i) {
      geometry_msgs::Point point;
      point.x = grid_tour[i][0];
      point.y = grid_tour[i][1];
      point.z = grid_tour[i][2];
      tour.points.push_back(point);
    }
    tour.drone_id = expl_manager_->ep_->drone_id_;
    tour.stamp = ros::Time::now().toSec();
    grid_tour_pub_.publish(tour);

    // visualization_->drawSpheres(
    //     expl_manager_->ed_->grid_tour_, 0.3, Eigen::Vector4d(0, 1, 0, 1), "grid_tour", 1, 6);
    // visualization_->drawLines(
    //     expl_manager_->ed_->grid_tour2_, 0.05, Eigen::Vector4d(0, 1, 0, 0.5), "grid_tour", 2, 6);

    // Top viewpoints and frontier tour-------------------------------------

    // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1), "point-average", 0, 6);

    // auto frontier = ed_ptr->frontier_tour_;
    // for (auto& pt : frontier) pt = pt + trans;
    // visualization_->drawLines(frontier, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "frontier_tour", 0, 6);

    // for (int i = 0; i < ed_ptr->other_tours_.size(); ++i) {
    //   visualization_->drawLines(
    //       ed_ptr->other_tours_[i], 0.07, Eigen::Vector4d(0, 0, 1, 1), "other_tours", i, 6);
    // }

    // Locally refined viewpoints and refined tour-------------------------------

    // visualization_->drawSpheres(
    //     ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05, Vector4d(0.5, 0, 1, 1),
    //     "refined_view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_tour_, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "refined_tour", 0, 6);

    // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0,
    // 0, 0, 1),
    //                           "refined_view", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05,
    // Vector4d(1, 1, 0, 1),
    //                           "refine_pair", 0, 6);
    // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
    //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
    //                               ed_ptr->frontiers_.size()),
    //                               "n_points", i, 6);
    // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
    //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

    // Trajectory-------------------------------------------

    // visualization_->drawSpheres(
    //     { ed_ptr->next_goal_ /* + trans */ }, 0.3, Vector4d(0, 0, 1, 1), "next_goal", 0, 6);

    // vector<Eigen::Vector3d> next_yaw_vis;
    // next_yaw_vis.push_back(ed_ptr->next_goal_ /* + trans */);
    // next_yaw_vis.push_back(
    //     ed_ptr->next_goal_ /* + trans */ +
    //     2.0 * Eigen::Vector3d(cos(ed_ptr->next_yaw_), sin(ed_ptr->next_yaw_), 0));
    // visualization_->drawLines(next_yaw_vis, 0.1, Eigen::Vector4d(0, 0, 1, 1), "next_goal", 1, 6);
    // visualization_->drawSpheres(
    //     { ed_ptr->next_pos_ /* + trans */ }, 0.3, Vector4d(0, 1, 0, 1), "next_pos", 0, 6);

    // Eigen::MatrixXd ctrl_pt = info->position_traj_.getControlPoint();
    // for (int i = 0; i < ctrl_pt.rows(); ++i) {
    //   for (int j = 0; j < 3; ++j) ctrl_pt(i, j) = ctrl_pt(i, j) + trans[j];
    // }
    // NonUniformBspline position_traj(ctrl_pt, 3, info->position_traj_.getKnotSpan());

    visualization_->drawBspline(info->position_traj_, 0.1,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        false, 0.15, Vector4d(1, 1, 0, 1));

    // visualization_->drawLines(
    //     expl_manager_->ed_->path_next_goal_, 0.1, Eigen::Vector4d(0, 1, 0, 1), "astar", 0, 6);
    // visualization_->drawSpheres(
    //     expl_manager_->ed_->kino_path_, 0.1, Eigen::Vector4d(0, 0, 1, 1), "kino", 0, 6);
    // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0,
    // 0); visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1),
    // "next_goal", 1, 6);

    // // Draw trajs of other drones
    // vector<NonUniformBspline> trajs;
    // planner_manager_->swarm_traj_data_.getValidTrajs(trajs);
    // for (int k = 0; k < trajs.size(); ++k) {
    //   visualization_->drawBspline(trajs[k], 0.1, Eigen::Vector4d(1, 1, 0, 1), false, 0.15,
    //       Eigen::Vector4d(0, 0, 1, 1), k + 1);
    // }
  }
}

void FastExplorationFSM::clearVisMarker() {
  for (int i = 0; i < 10; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "dead_frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  if (state_ == WAIT_TRIGGER) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;

    auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
      double a = (drone_id - 1) / double(num + 1);
      double b = 1 / double(num + 1);
      return a + b * double(id) / ed->frontiers_.size();
    };

    // ft->searchFrontiers();
    // ft->computeFrontiersToVisit();
    // ft->updateFrontierCostMatrix();

    // ft->getFrontiers(ed->frontiers_);
    // ft->getFrontierBoxes(ed->frontier_boxes_);

    expl_manager_->updateFrontierStruct(fd_->odom_pos_);

    cout << "odom: " << fd_->odom_pos_.transpose() << endl;
    vector<int> tmp_id1;
    vector<vector<int>> tmp_id2;
    bool status = expl_manager_->findGlobalTourOfGrid(
        { fd_->odom_pos_ }, { fd_->odom_vel_ }, tmp_id1, tmp_id2, true);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4), "frontier", i, 4);
      // getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
      // double(i) / ed->frontiers_.size()
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    if (status)
      visualize(2);
    else
      visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);

    // Draw grid tour
  }
}

void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  // // Debug traj planner
  // Eigen::Vector3d pos;
  // pos << msg->pose.position.x, msg->pose.position.y, 1;
  // expl_manager_->ed_->next_pos_ = pos;

  // Eigen::Vector3d dir = pos - fd_->odom_pos_;
  // expl_manager_->ed_->next_yaw_ = atan2(dir[1], dir[0]);
  // fd_->go_back_ = true;
  // transitState(PLAN_TRAJ, "triggerCallback");
  // return;

  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  fd_->start_pos_ = fd_->odom_pos_;
  ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());

  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
    transitState(PLAN_TRAJ, "triggerCallback");
  } else
    transitState(FINISH, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "safetyCallback");
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

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = ros::Time::now();
  }
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  ROS_INFO_STREAM("[" + pos_call + "]: Drone "
                  << getId()
                  << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}

void FastExplorationFSM::droneStateTimerCallback(const ros::TimerEvent& e) {
  // Broadcast own state periodically
  exploration_manager::DroneState msg;
  msg.drone_id = getId();

  auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  if (fd_->static_state_) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  } else {
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (ros::Time::now() - info->start_time_).toSec();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }
  state.stamp_ = ros::Time::now().toSec();
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.yaw = state.yaw_;
  for (auto id : state.grid_ids_) msg.grid_ids.push_back(id);
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;

  drone_state_pub_.publish(msg);
}

void FastExplorationFSM::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg) {
  // Update other drones' states
  if (msg->drone_id == getId()) return;

  // Simulate swarm communication loss
  Eigen::Vector3d msg_pos(msg->pos[0], msg->pos[1], msg->pos[2]);
  // if ((msg_pos - fd_->odom_pos_).norm() > 6.0) return;

  auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;  // Avoid unordered msg

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;
  drone_state.grid_ids_.clear();
  for (auto id : msg->grid_ids) drone_state.grid_ids_.push_back(id);
  drone_state.stamp_ = msg->stamp;
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;

  // std::cout << "Drone " << getId() << " get drone " << int(msg->drone_id) << "'s state" <<
  // std::endl; std::cout << drone_state.pos_.transpose() << std::endl;
}

void FastExplorationFSM::optTimerCallback(const ros::TimerEvent& e) {
  if (state_ == INIT) return;

  // Select nearby drone not interacting with recently
  auto& states = expl_manager_->ed_->swarm_state_;
  auto& state1 = states[getId() - 1];
  // bool urgent = (state1.grid_ids_.size() <= 1 /* && !state1.grid_ids_.empty() */);
  bool urgent = state1.grid_ids_.empty();
  auto tn = ros::Time::now().toSec();

  // Avoid frequent attempt
  if (tn - state1.recent_attempt_time_ < fp_->attempt_interval_) return;

  int select_id = -1;
  double max_interval = -1.0;
  for (int i = 0; i < states.size(); ++i) {
    if (i + 1 <= getId()) continue;
    // Check if have communication recently
    // or the drone just experience another opt
    // or the drone is interacted with recently /* !urgent &&  */
    // or the candidate drone dominates enough grids
    if (tn - states[i].stamp_ > 0.2) continue;
    if (tn - states[i].recent_attempt_time_ < fp_->attempt_interval_) continue;
    if (tn - states[i].recent_interact_time_ < fp_->pair_opt_interval_) continue;
    if (states[i].grid_ids_.size() + state1.grid_ids_.size() == 0) continue;

    double interval = tn - states[i].recent_interact_time_;
    if (interval <= max_interval) continue;
    select_id = i + 1;
    max_interval = interval;
  }
  if (select_id == -1) return;

  std::cout << "\nSelect: " << select_id << std::endl;
  ROS_WARN("Pair opt %d & %d", getId(), select_id);

  // Do pairwise optimization with selected drone, allocate the union of their domiance grids
  unordered_map<int, char> opt_ids_map;
  auto& state2 = states[select_id - 1];
  for (auto id : state1.grid_ids_) opt_ids_map[id] = 1;
  for (auto id : state2.grid_ids_) opt_ids_map[id] = 1;
  vector<int> opt_ids;
  for (auto pair : opt_ids_map) opt_ids.push_back(pair.first);

  std::cout << "Pair Opt id: ";
  for (auto id : opt_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Find missed grids to reallocated them
  vector<int> actives, missed;
  expl_manager_->hgrid_->getActiveGrids(actives);
  findUnallocated(actives, missed);
  std::cout << "Missed: ";
  for (auto id : missed) std::cout << id << ", ";
  std::cout << "" << std::endl;
  opt_ids.insert(opt_ids.end(), missed.begin(), missed.end());

  // Do partition of the grid
  vector<Eigen::Vector3d> positions = { state1.pos_, state2.pos_ };
  vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0) };
  vector<int> first_ids1, second_ids1, first_ids2, second_ids2;
  if (state_ != WAIT_TRIGGER) {
    expl_manager_->hgrid_->getConsistentGrid(
        state1.grid_ids_, state1.grid_ids_, first_ids1, second_ids1);
    expl_manager_->hgrid_->getConsistentGrid(
        state2.grid_ids_, state2.grid_ids_, first_ids2, second_ids2);
  }

  auto t1 = ros::Time::now();

  vector<int> ego_ids, other_ids;
  expl_manager_->allocateGrids(positions, velocities, { first_ids1, first_ids2 },
      { second_ids1, second_ids2 }, opt_ids, ego_ids, other_ids);

  double alloc_time = (ros::Time::now() - t1).toSec();

  std::cout << "Ego1  : ";
  for (auto id : state1.grid_ids_) std::cout << id << ", ";
  std::cout << "\nOther1: ";
  for (auto id : state2.grid_ids_) std::cout << id << ", ";
  std::cout << "\nEgo2  : ";
  for (auto id : ego_ids) std::cout << id << ", ";
  std::cout << "\nOther2: ";
  for (auto id : other_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Check results
  double prev_app1 = expl_manager_->computeGridPathCost(state1.pos_, state1.grid_ids_, first_ids1,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double prev_app2 = expl_manager_->computeGridPathCost(state2.pos_, state2.grid_ids_, first_ids2,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  std::cout << "prev cost: " << prev_app1 << ", " << prev_app2 << ", " << prev_app1 + prev_app2
            << std::endl;
  double cur_app1 = expl_manager_->computeGridPathCost(state1.pos_, ego_ids, first_ids1,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double cur_app2 = expl_manager_->computeGridPathCost(state2.pos_, other_ids, first_ids2,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  std::cout << "cur cost : " << cur_app1 << ", " << cur_app2 << ", " << cur_app1 + cur_app2
            << std::endl;
  if (cur_app1 + cur_app2 > prev_app1 + prev_app2 + 0.1) {
    ROS_ERROR("Larger cost after reallocation");
    if (state_!=WAIT_TRIGGER) {
      return;
    }
  }

  if (!state1.grid_ids_.empty() && !ego_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(state1.grid_ids_[0], ego_ids[0])) {
    ROS_ERROR("Path 1 inconsistent");
  }
  if (!state2.grid_ids_.empty() && !other_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(state2.grid_ids_[0], other_ids[0])) {
    ROS_ERROR("Path 2 inconsistent");
  }

  // Update ego and other dominace grids
  auto last_ids2 = state2.grid_ids_;

  // Send the result to selected drone and wait for confirmation
  exploration_manager::PairOpt opt;
  opt.from_drone_id = getId();
  opt.to_drone_id = select_id;
  // opt.msg_type = 1;
  opt.stamp = tn;
  for (auto id : ego_ids) opt.ego_ids.push_back(id);
  for (auto id : other_ids) opt.other_ids.push_back(id);

  for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_pub_.publish(opt);

  ROS_WARN("Drone %d send opt request to %d, pair opt t: %lf, allocate t: %lf", getId(), select_id,
      ros::Time::now().toSec() - tn, alloc_time);

  // Reserve the result and wait...
  auto ed = expl_manager_->ed_;
  ed->ego_ids_ = ego_ids;
  ed->other_ids_ = other_ids;
  ed->pair_opt_stamp_ = opt.stamp;
  ed->wait_response_ = true;
  state1.recent_attempt_time_ = tn;
}

void FastExplorationFSM::findUnallocated(const vector<int>& actives, vector<int>& missed) {
  // Create map of all active
  unordered_map<int, char> active_map;
  for (auto ativ : actives) {
    active_map[ativ] = 1;
  }

  // Remove allocated ones
  for (auto state : expl_manager_->ed_->swarm_state_) {
    for (auto id : state.grid_ids_) {
      if (active_map.find(id) != active_map.end()) {
        active_map.erase(id);
      } else {
        // ROS_ERROR("Inactive grid %d is allocated.", id);
      }
    }
  }

  missed.clear();
  for (auto p : active_map) {
    missed.push_back(p.first);
  }
}

void FastExplorationFSM::optMsgCallback(const exploration_manager::PairOptConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

  // Check stamp to avoid unordered/repeated msg
  if (msg->stamp <= expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] + 1e-4) return;
  expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto& state1 = expl_manager_->ed_->swarm_state_[msg->from_drone_id - 1];
  auto& state2 = expl_manager_->ed_->swarm_state_[getId() - 1];

  // auto tn = ros::Time::now().toSec();
  exploration_manager::PairOptResponse response;
  response.from_drone_id = msg->to_drone_id;
  response.to_drone_id = msg->from_drone_id;
  response.stamp = msg->stamp;  // reply with the same stamp for verificaiton

  if (msg->stamp - state2.recent_attempt_time_ < fp_->attempt_interval_) {
    // Just made another pair opt attempt, should reject this attempt to avoid frequent changes
    ROS_WARN("Reject frequent attempt");
    response.status = 2;
  } else {
    // No opt attempt recently, and the grid info between drones are consistent, the pair opt
    // request can be accepted
    response.status = 1;

    // Update from the opt result
    state1.grid_ids_.clear();
    state2.grid_ids_.clear();
    for (auto id : msg->ego_ids) state1.grid_ids_.push_back(id);
    for (auto id : msg->other_ids) state2.grid_ids_.push_back(id);

    state1.recent_interact_time_ = msg->stamp;
    state2.recent_attempt_time_ = ros::Time::now().toSec();
    expl_manager_->ed_->reallocated_ = true;

    if (state_ == IDLE && !state2.grid_ids_.empty()) {
      transitState(PLAN_TRAJ, "optMsgCallback");
      ROS_WARN("Restart after opt!");
    }

    // if (!check_consistency(tmp1, tmp2)) {
    //   response.status = 2;
    //   ROS_WARN("Inconsistent grid info, reject pair opt");
    // } else {
    // }
  }
  for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_res_pub_.publish(response);
}

void FastExplorationFSM::optResMsgCallback(
    const exploration_manager::PairOptResponseConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

  // Check stamp to avoid unordered/repeated msg
  if (msg->stamp <= expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] + 1e-4) return;
  expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto ed = expl_manager_->ed_;
  // Verify the consistency of pair opt via time stamp
  if (!ed->wait_response_ || fabs(ed->pair_opt_stamp_ - msg->stamp) > 1e-5) return;

  ed->wait_response_ = false;
  ROS_WARN("get response %d", int(msg->status));

  if (msg->status != 1) return;  // Receive 1 for valid opt

  auto& state1 = ed->swarm_state_[getId() - 1];
  auto& state2 = ed->swarm_state_[msg->from_drone_id - 1];
  state1.grid_ids_ = ed->ego_ids_;
  state2.grid_ids_ = ed->other_ids_;
  state2.recent_interact_time_ = ros::Time::now().toSec();
  ed->reallocated_ = true;

  if (state_ == IDLE && !state1.grid_ids_.empty()) {
    transitState(PLAN_TRAJ, "optResMsgCallback");
    ROS_WARN("Restart after opt!");
  }
}

void FastExplorationFSM::swarmTrajCallback(const bspline::BsplineConstPtr& msg) {
  // Get newest trajs from other drones, for inter-drone collision avoidance
  auto& sdat = planner_manager_->swarm_traj_data_;

  // Ignore self trajectory
  if (msg->drone_id == sdat.drone_id_) return;

  // Ignore outdated trajectory
  if (sdat.receive_flags_[msg->drone_id - 1] == true &&
      msg->start_time.toSec() <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
    return;

  // Convert the msg to B-spline
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  // // Transform of drone's basecoor, optional step (skip if use swarm_pilot)
  // Eigen::Vector4d tf;
  // planner_manager_->edt_environment_->sdf_map_->getBaseCoor(msg->drone_id, tf);
  // double yaw = tf[3];
  // Eigen::Matrix3d rot;
  // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // Eigen::Vector3d trans = tf.head<3>();
  // for (int i = 0; i < pos_pts.rows(); ++i) {
  //   Eigen::Vector3d tmp = pos_pts.row(i);
  //   tmp = rot * tmp + trans;
  //   pos_pts.row(i) = tmp;
  // }

  sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
  sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
  sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = msg->start_time.toSec();
  sdat.receive_flags_[msg->drone_id - 1] = true;

  if (state_ == EXEC_TRAJ) {
    // Check collision with received trajectory
    if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
      ROS_ERROR("Drone %d collide with drone %d.", sdat.drone_id_, msg->drone_id);
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "swarmTrajCallback");
    }
  }
}

void FastExplorationFSM::swarmTrajTimerCallback(const ros::TimerEvent& e) {
  // Broadcast newest traj of this drone to others
  if (state_ == EXEC_TRAJ) {
    swarm_traj_pub_.publish(fd_->newest_traj_);

  } else if (state_ == WAIT_TRIGGER) {
    // Publish a virtual traj at current pose, to avoid collision
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = ros::Time::now();
    bspline.traj_id = planner_manager_->local_data_.traj_id_;

    Eigen::MatrixXd pos_pts(4, 3);
    for (int i = 0; i < 4; ++i) pos_pts.row(i) = fd_->odom_pos_.transpose();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
    Eigen::VectorXd knots = tmp.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    bspline.drone_id = expl_manager_->ep_->drone_id_;
    swarm_traj_pub_.publish(bspline);
  }
}

}  // namespace fast_planner
