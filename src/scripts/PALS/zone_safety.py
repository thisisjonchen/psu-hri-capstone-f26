"""Check surveyed rectangular zones against walls and obstacles.

Coordinates and uncertainty are in meters in one shared map frame.
This module makes a decision only; it never commands a drone.
"""

import argparse
import json
import math


CLEARANCE_M = 3.048  # Exactly 10 feet.


def rectangle(value):
    """Rectangle format: [xmin, ymin, xmax, ymax]."""
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError("A rectangle needs four coordinates")

    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError("Coordinates must be finite")

    xmin, ymin, xmax, ymax = result
    if xmin >= xmax or ymin >= ymax:
        raise ValueError("Rectangle must have positive width and height")

    return result


def contained(inner, outer):
    return (
        outer[0] <= inner[0]
        and outer[1] <= inner[1]
        and inner[2] <= outer[2]
        and inner[3] <= outer[3]
    )


def distance(a, b):
    """Shortest horizontal distance between rectangle edges."""
    dx = max(a[0] - b[2], b[0] - a[2], 0.0)
    dy = max(a[1] - b[3], b[1] - a[3], 0.0)
    return math.hypot(dx, dy)


def evaluate_survey(survey):
    """Return PROCEED or ABORT. Invalid/incomplete input fails closed.

    Obstacles must include all relevant obstructions from both cameras.
    Conservative map uncertainty increases the required clearance.
    """
    reasons = []

    try:
        environment = rectangle(survey["environment"])
        zones = {
            name: rectangle(survey["zones"][name])
            for name in ("pickup", "delivery")
        }

        uncertainty = float(survey["max_map_error_m"])
        if not math.isfinite(uncertainty) or uncertainty < 0:
            raise ValueError("Map error must be finite and nonnegative")

        # Required explicit evidence flags from the survey pipeline.
        for flag in (
            "survey_complete",
            "clearance_areas_fully_observed",
            "metric_map_valid",
            "both_camera_streams_valid",
        ):
            if survey[flag] is not True:
                reasons.append("Survey requirement failed: " + flag)

        obstacles = survey["obstacles"]
        if not isinstance(obstacles, list):
            raise ValueError("obstacles must be a list")

        # Validate every obstacle before allowing approval.
        mapped_obstacles = []
        for obstacle in obstacles:
            name = str(obstacle["id"])
            bounds = rectangle(obstacle["bounds"])
            if not contained(bounds, environment):
                raise ValueError("Obstacle outside environment: " + name)
            mapped_obstacles.append((name, bounds))

        required = CLEARANCE_M + uncertainty

        for zone_name, zone in zones.items():
            if not contained(zone, environment):
                raise ValueError(zone_name + " zone outside environment")

            # The rectangular environment boundary represents four walls.
            wall_distance = min(
                zone[0] - environment[0],
                zone[1] - environment[1],
                environment[2] - zone[2],
                environment[3] - zone[3],
            )
            if wall_distance <= required:
                reasons.append(
                    "{} zone: wall clearance {:.3f} m "
                    "is within required {:.3f} m".format(
                        zone_name, wall_distance, required
                    )
                )

            for obstacle_name, bounds in mapped_obstacles:
                gap = distance(zone, bounds)
                if gap <= required:
                    reasons.append(
                        "{} zone: obstacle '{}' at {:.3f} m "
                        "is within required {:.3f} m".format(
                            zone_name, obstacle_name, gap, required
                        )
                    )

    except (KeyError, TypeError, ValueError, OverflowError) as error:
        reasons.append("Invalid or missing survey data: " + str(error))

    return {
        "decision": "ABORT" if reasons else "PROCEED",
        "reasons": reasons,
        "clearance_m": CLEARANCE_M,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("survey_file")
    args = parser.parse_args()

    try:
        with open(args.survey_file, "r") as source:
            result = evaluate_survey(json.load(source))
    except (OSError, ValueError) as error:
        result = {
            "decision": "ABORT",
            "reasons": ["Cannot read survey: " + str(error)],
            "clearance_m": CLEARANCE_M,
        }

    print(json.dumps(result, indent=2))
    return 2 if result["decision"] == "ABORT" else 0


if __name__ == "__main__":
    raise SystemExit(main())