use std::collections::HashSet;

use geo::{
    line_intersection::line_intersection, Coordinate, CoordsIter, Line, LineInterpolatePoint,
    LineString, RotatePoint,
};
use rand_distr::num_traits::Pow;
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator, ParallelBridge};

use super::{DynamicObstacle, Enviroment, Obstacles};

pub fn find_collision_point<'a>(
    path: &LineString,
    obstacles: &'a Obstacles,
    enviroment: &Enviroment,
) -> Vec<(Coordinate, &'a DynamicObstacle)> {
    
    let projection_line_len = f64::sqrt(enviroment.width.pow(2) + enviroment.height.pow(2));
    let (sender, receiver) = flume::unbounded();
    obstacles.dynamic_obstacles.par_iter().for_each(|dyn_obstacle| {
        dyn_obstacle
            .safe_sphere
            .exterior_coords_iter()
            .par_bridge()
            .for_each_with(sender.clone(), |s, coord| {
                let line = Line::new(coord, coord + [0.0, projection_line_len].into());
                let line = line.rotate_around_point(dyn_obstacle.course, coord.into());
                path.lines().for_each(|path_line| {
                    if let Some(x) = line_intersection(line, path_line) {
                        match x {
                            geo::LineIntersection::SinglePoint {
                                intersection,
                                is_proper,
                            } => s.send((intersection, dyn_obstacle)).unwrap(),
                            geo::LineIntersection::Collinear { intersection } => {
                                s.send((intersection.start, dyn_obstacle)).unwrap();
                                s.send((intersection.end,   dyn_obstacle)).unwrap();
                            }
                        }
                    }
                });
            });
    });
    drop(sender);
    let result: Vec<(Coordinate, &DynamicObstacle)> = receiver.iter().collect();
    result
}
