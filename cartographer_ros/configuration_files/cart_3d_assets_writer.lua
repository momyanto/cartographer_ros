VOXEL_SIZE = 5e-2

include "transform.lua"

options = {
  tracking_frame = "imu_link",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 2.,
      max_range = 60.,
    },
    {
      action = "dump_num_points",
    },
    {
      action = "fixed_ratio_sampler",
      sampling_ratio=0.2,
    },
    -- {
    --   action = "intensity_to_color",
    --   min_intensity = 0.,
    --   max_intensity = 255.,
    -- },
    {
      action = "write_pcd",
      filename = "map.pcd"
    },
    -- {
    --   action = "write_ply",
    --   filename = "map_sampling_0.2.ply"
    -- },
    {
      action = "write_ros_map",
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      filestem = "map",
      resolution = 0.05,
    }
  }
}

return options