use std::collections::HashSet;

use geo::{
    line_intersection::line_intersection, Coordinate, CoordsIter, EuclideanLength, Line,
    LineInterpolatePoint, LineString, RotatePoint, MultiLineString, line_string, Translate, Point,
};
use geo_clipper::{Clipper, ClipperOpen};
use rand_distr::num_traits::Pow;
use rayon::prelude::{IntoParallelRefIterator, ParallelBridge, ParallelIterator};

use crate::draw_env_to_file;

use super::{DynamicObstacle, Enviroment, Individual, Obstacles, GeneticAlgorithm};

pub fn find_collision_point<'a>(
    path: &LineString,
    obstacles: &'a Obstacles,
    enviroment: &Enviroment,
) -> Vec<(usize, Coordinate, &'a DynamicObstacle)> {
    let projection_line_len = f64::sqrt(enviroment.width.pow(2) + enviroment.height.pow(2));
    let (sender, receiver) = flume::unbounded();
    obstacles
        .dynamic_obstacles
        .par_iter()
        .for_each(|dyn_obstacle| {
            dyn_obstacle
                .safe_sphere
                .exterior_coords_iter()
                .par_bridge()
                .for_each_with(sender.clone(), |s, coord| {
                    let line = Line::new(coord, coord + [0.0, projection_line_len].into());
                    let line = line.rotate_around_point(dyn_obstacle.course, coord.into());
                    path.lines().enumerate().for_each(|(idx, path_line)| {
                        if let Some(x) = line_intersection(line, path_line) {
                            match x {
                                geo::LineIntersection::SinglePoint {
                                    intersection,
                                    is_proper,
                                } => s.send((idx, intersection, dyn_obstacle)).unwrap(),
                                geo::LineIntersection::Collinear { intersection } => {
                                    s.send((idx, intersection.start, dyn_obstacle)).unwrap();
                                    s.send((idx, intersection.end, dyn_obstacle)).unwrap();
                                }
                            }
                        }
                    });
                });
        });
    drop(sender);
    let mut result: Vec<(usize, Coordinate, &'a DynamicObstacle)> = receiver.iter().collect();
    result.sort_by(|a, b| a.0.cmp(&b.0));
    result.dedup_by(|a,b| a.1.eq(&b.1));
    result
}

pub fn check_collision_with_dynamic_obstacles(
    ga: &GeneticAlgorithm,
    crossing_points: &Vec<(usize, Coordinate<f64>, &DynamicObstacle)>,
    individual: &Individual,
) -> MultiLineString {
    let mut result: MultiLineString = MultiLineString::new(vec![]);
    let mut current_path_time = 0.;
    let mut line_iter = individual.points.windows(2).enumerate();
    let mut line_idx = 0usize;
    let mut current_line: MultiLineString = {
        let (_,line) = line_iter.next().unwrap();
        MultiLineString::new(vec![line_string![line[0], line[1]]])
    };
    for crossing_point in crossing_points{

        while line_idx != crossing_point.0{
            let temp = line_iter.next().unwrap();
            current_path_time += individual.speed[line_idx] * current_line.euclidean_length();
            current_line.0[0] = line_string![temp.1[0], temp.1[1]];
            line_idx = temp.0;
        }
        let current_line_path_time = individual.speed[line_idx] * Line::new(current_line.0[0].0.last().unwrap().clone(), crossing_point.1).euclidean_length();
        let dyn_obstacle_travel_distance = (current_path_time + current_line_path_time) * crossing_point.2.speed;
        let translation_vector = Point::new(0., dyn_obstacle_travel_distance)
        .rotate_around_point(crossing_point.2.course, [0.0, 0.0].into());

        let moved_obstacle = crossing_point.2.safe_sphere.translate(translation_vector.x(), translation_vector.y());
        let intersection = current_line.intersection(&moved_obstacle, 1000.);
        if intersection.euclidean_length() > 0.{
            // draw_env_to_file(
            //     "eval_with_dyn_obs.png",
            //     &ga.obstacles,
            //     &ga.enviroment,
            //     &individual.points,
            //     Some(crossing_points.iter().map(|x| x.1).collect()),
            //     Some(intersection.clone()),
            //     Some(moved_obstacle),
            // ).unwrap();
        }

        result.0.extend(intersection);
    }
    result
}
