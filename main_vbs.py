from vbs_parser import parse_config, VBSConfig
from vbs_environment import Environment
from vbs_solver import VisibilityBasedSolver
import os

def main():
    # Ensure the output directory for C++ might exist, or create one for Python outputs
    # The Python classes will create "./output_py/" if needed.
    # If we want to strictly mimic C++ that uses "./output/", we can adjust paths in classes
    # or ensure it's created here. For now, Python classes handle their own output dir.

    print("Starting Python VBS planner...")

    # Parse settings
    # Assuming the config file is in a 'config' directory relative to this script,
    # similar to the C++ structure.
    # config_file_path = os.path.join("config", "settings.config")

    # Hardcode config for testing to avoid file issues and ensure quick run
    print("Using hardcoded configuration for main_vbs.py test run.")
    config = VBSConfig(
        mode=1, ncols=30, nrows=20, nb_of_obstacles=5,
        min_width=2, max_width=4, min_height=2, max_height=4, # Corrected maxHeight
        random_seed=False, seed_value=42,
        # start=(2,2), end=(28,18), # These are set below correctly as (x,y)
        max_iter=30, visibility_threshold=0.3, light_strength=1.0,
        timer=True, save_results=False,
        save_local_visibility = False, save_came_from = False, save_light_sources = False, # Corrected case
        save_global_visibility = False, save_visibility_field = False,
        silent=False, ball_radius=1
    )
    # Ensure VBSConfig start/end are (x,y) as expected by its parser/structure
    # The VBSConfig default is (0,0) but if we set it, it's direct.
    # parse_pair_string returns (int(parts[0]), int(parts[1])) which is (x,y) from "{x,y}"
    # So config.start = (x,y)
    # Environment uses config.start directly.
    # Solver uses config.start directly.
    # Let's ensure our tuple is (x,y) for consistency with how VBSConfig would parse it.
    # Start point (x=2, y=2), End point (x=28, y=18)
    config.start = (2,2)
    config.end = (28,18)


    if config is None: # Should not happen with hardcoded config
        print("Exiting due to configuration parsing failure.")
        return 1

    # Initialize environment
    # The Environment class constructor handles mode 1 (random) or mode 2 (image)
    # and associated file saving if configured.
    print("\nInitializing Environment...")
    env = Environment(config)

    if env.nx == 0 or env.ny == 0:
        print("Environment initialization failed or resulted in an empty grid. Exiting.")
        return 1

    print(f"Environment initialized: {env.nx}x{env.ny}")

    # Initialize solver & solve
    print("\nInitializing VisibilityBasedSolver...")
    solver = VisibilityBasedSolver(env)
    print("Solver initialized.")

    print("\nAttempting to solve...")
    # The solve() method in Python will handle printing path found/not found
    # and saving results if configured.
    resulting_path = solver.solve()

    # StandAloneVisibility and Benchmark calls (optional, based on C++ main)
    # These are useful for debugging and comparison.
    # Let's make them conditional or less prominent for a typical run.

    run_extra_analysis = False # Set to True to run these analysis steps
    if config.save_results: # Only run if saving is generally enabled
        run_extra_analysis = True

    if run_extra_analysis:
        print("\nRunning stand-alone visibility analysis...")
        solver.stand_alone_visibility() # Uses config.start as light source

        print("\nRunning benchmark against raycasting...")
        solver.benchmark() # Compares _compute_visibility with raycasting

        # solver.benchmark_series() # This can be very slow, usually run separately.
        # print("\nTo run benchmark series, uncomment the call in main_vbs.py")

    print("\nPython VBS planner finished.")
    return 0

if __name__ == '__main__':
    main()
