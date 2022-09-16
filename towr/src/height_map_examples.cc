/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
#include <cmath>
#include <towr/terrain/examples/height_map_examples.h>

namespace towr {


FlatGround::FlatGround(double height)
{
  height_ = height;
}

double
Block::GetHeight (double x, double y) const
{
  double h = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    h = slope_*(x-block_start);

  if (block_start+eps_ <= x && x <= block_start+length_)
    h = height_;

  return h;
}

double
Block::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    dhdx = slope_;

  return dhdx;
}


// STAIRS
double
Stairs::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (x>=first_step_start_)
    h = height_first_step;

  if (x>=first_step_start_+first_step_width_)
    h = height_second_step;

  if (x>=first_step_start_+first_step_width_+width_top)
    h = 0.0;

  return h;
}


// GAP
double
Gap::GetHeight (double x, double y) const
{
  double h = 0.0;

  // modelled as parabola
  if (gap_start_ <= x && x <= gap_end_x)
    h = a*x*x + b*x + c;

  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dhdx = 2*a*x + b;

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dzdxx = 2*a;

  return dzdxx;
}


// SLOPE
double
Slope::GetHeight (double x, double y) const
{
  double z = 0.0;
  if (x >= slope_start_)
    z = slope_*(x-slope_start_);

  // going back down
  if (x >= x_down_start_) {
    z = height_center - slope_*(x-x_down_start_);
  }

  // back on flat ground
  if (x >= x_flat_start_)
    z = 0.0;

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  if (x >= x_down_start_)
    dzdx = -slope_;

  if (x >= x_flat_start_)
    dzdx = 0.0;

  return dzdx;
}


// Chimney
double
Chimney::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end_)
    z = slope_*(y-y_start_);

  return z;
}

double
Chimney::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_<= x && x<= x_end_)
    dzdy = slope_;

  return dzdy;
}


// Chimney LR
double
ChimneyLR::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end1_)
    z = slope_*(y-y_start_);

  if (x_end1_<=x && x<=x_end2_)
    z = -slope_*(y+y_start_);

  return z;
}

double
ChimneyLR::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_ <= x && x <= x_end1_)
    dzdy = slope_;

  if (x_end1_<=x && x<=x_end2_)
    dzdy = -slope_;

  return dzdy;
}

// GTSTAIRS
double
GTStairs::GetHeight (double x, double y) const
{
  double h = 0.0;
  double down_step_start_ = step_start_ + (n_steps-1)*step_width_ + width_top_;
  if ( (x - step_start_) / step_width_ < n_steps-1 
    && x >= step_start_)
    h = step_height_ * int((x - step_start_) / step_width_ +1);

  if (x >= step_start_ + (n_steps-1)*step_width_ 
    && x <= down_step_start_)
    h = step_height_ * n_steps;

  if (x > down_step_start_
    && x <= down_step_start_ + (n_steps-1)*step_width_)
    h = step_height_*int(n_steps - (x - down_step_start_) / step_width_);

  return h;
}

// GTSTAIRS2
double
GTStairs2::GetHeight (double x, double y) const
{
  // GTStairs2 is designed so that actual stairs and touchable region are different.
  // We need this because feet steping too close to next step leads to feet penatration. 
  // Basic idea is to make "stair edges" untraversable.

  double h = 0.0;
  if (x < step_start_ - step_margin_jump_)
    return 0.0;

  if (x < step_start_)
    return penalty_height_;
    
  // after stair start
  double distance_from_stair_start = x - step_start_;
  // Before start
  if (distance_from_stair_start < -step_margin_jump_)
    return 0.0;
  
  // Starting margin
  if (distance_from_stair_start < 0)
    return penalty_height_;


  int current_stair_lvl = std::floor(distance_from_stair_start/ step_width_) + 1;
  
  // reach max step
  if (current_stair_lvl >= n_steps) 
    return step_height_ * n_steps;
  
  // set untraversable region
  double distance_from_step_start = (distance_from_stair_start) - (current_stair_lvl-1) * step_width_;
  double distance_to_next_step    = step_width_ - distance_from_step_start;
  if ((distance_from_step_start > step_margin_land_) && 
      (distance_from_step_start < step_width_ - step_margin_jump_))
    return step_height_ * current_stair_lvl;
  
  return penalty_height_;
}

// Obstacle1
double
Obstacle1::GetHeight (double x, double y) const
{
  double h = 0.0;

  // modelled as parabola
  double center_distance = (x-center_x)*(x-center_x) + (y-center_y)*(y-center_y);
  if ( center_distance <= radius * radius)
    h = height_;

  return h;
}


// Obstacle2
double
Obstacle2::GetHeight (double x, double y) const
{
  double h = 0.0;

  // modelled as parabola
  double center_distance = (x-center_x)*(x-center_x) + (y-center_y)*(y-center_y);
  if ( center_distance <= radius * radius)
    h = height_;

  return h;
}


} /* namespace towr */
