#include "motion_planner.h"
#include <cstddef>

MotionPlanner::MotionPlanner(const Config& config)
{
}

void MotionPlanner::init()
{
}

void MotionPlanner::reset()
{
}

void MotionPlanner::update()
{

}

void MotionPlanner::connect_inputs(const TrackPath* path)
{
    input_path_ = path;
}
