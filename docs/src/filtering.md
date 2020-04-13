# Particle Filtering

These functions are responsible for performing particle filtering for parameter estimation.

```@docs
    c_idm_from_particle
    hallucinate_a_step
    weight_and_resample
    multistep_update
    obtain_driver_models
```

The following functions are helpers. They provide access to information about vehicle state, as well as help visualize filtering progress.

```@docs
    get_frenet_s
    get_veh_info
    get_lane_id
    get_lane_change_prob
    initial_pmat
    pgfplots2gif
    plot_pairwise_particles
```