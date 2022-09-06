use std::sync::mpsc::channel;

use geo::{
    Closest, ClosestPoint, Contains, Coordinate, CoordsIter, EuclideanLength, Intersects, Line,
    LineString, MultiLineString, line_string,
};
use geo_clipper::ClipperOpen;
use petgraph::algo::astar;
use rand::{prelude::SliceRandom, thread_rng, random};
use rand_distr::{Distribution, Uniform, UnitCircle, UnitDisc};
use rayon::iter::{IntoParallelRefIterator, ParallelBridge, ParallelIterator};

use crate::{draw_env_to_file, ga::dynamic::{find_collision_point, check_collision_with_dynamic_obstacles}};

use super::{GeneticAlgorithm, Individual};

pub fn hard_mutation(ga: &GeneticAlgorithm, individual: &mut Individual) {
    // println!("hard_mutation");
    let mut rng = thread_rng();
    let len = individual.points.len();
    let env_width_range = Uniform::new(0., ga.enviroment.width);
    let env_height_range = Uniform::new(0., ga.enviroment.height);
    let mut element = individual.points[1..(len - 1)]
        .choose_mut(&mut rng)
        .unwrap();
    element.x = env_width_range.sample(&mut rng);
    element.y = env_height_range.sample(&mut rng);

}
#[allow(unused_variables)]
pub fn swap_mutation(ga: &GeneticAlgorithm, individual: &mut Individual) {
    if individual.points.len() <= 3 {
        // println!("swap_mutation z 3p");
        return;
    }
    let mut rng = thread_rng();
    if individual.points.len() <= 2 {
        return;
    }
    let range = Uniform::from(1..(individual.points.len() - 1));
    let (mut p1, mut p2) = (0, 0);

    while p1 == p2 {
        p1 = range.sample(&mut rng);
        p2 = range.sample(&mut rng);
    }

    individual.points.swap(p1, p2);
    individual.speed.swap(p1, p2);
}

pub fn move_mutation(ga: &GeneticAlgorithm, individual: &mut Individual) {
    // println!("move_mutation");
    let mut rng = thread_rng();
    // let range = Uniform::new(0., 1.);
    let mut run_once = false;
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    let mut result_speed: Vec<f64> = Vec::with_capacity(individual.points.len());
    result.push(individual.points.first().unwrap().clone());
    result_speed.push(individual.speed.first().unwrap().clone());
    for (idx, point) in individual.points[1..(individual.points.len() - 1)].iter().enumerate() {
        for poly in ga.obstacles.static_obstacles.iter() {
            if !run_once && poly.contains(point) {
                run_once = true;
                // let min_dist = poly.exterior().euclidean_distance(&Point::new(point.x, point.y));
                let closest = match poly.exterior().closest_point(&point.clone().into()) {
                    Closest::Intersection(val) => val,
                    Closest::SinglePoint(val) => val,
                    Closest::Indeterminate => panic!("die i guess?"),
                };

                let random_point_on_circle: [f64; 2] = UnitCircle.sample(&mut rng);
                result.push(
                    (closest.x() + random_point_on_circle[0], closest.y() + random_point_on_circle[1],).into(),
                );
                result_speed.push(ga.config.individual_speed_values.choose(&mut rng).unwrap().clone());
                break;
            }
        }
        result.push(point.clone());
        result_speed.push(individual.speed[idx + 1]);
    }
    result.push(individual.points.last().unwrap().clone());
    result_speed.push(individual.speed.last().unwrap().clone());
    assert!(result.len() == result_speed.len());
    individual.points = result;
    individual.speed = result_speed;
}

/// This mutation loops over all points and rolls a 10% chance to remove it if it is not inside any polygons.
pub fn delete_mutation(ga: &GeneticAlgorithm, individual: &mut Individual) {
    if individual.points.len() < 5 {
        return;
    }
    let mut rng = thread_rng();
    let range = Uniform::new(0., 1.);
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    let mut result_speed: Vec<f64> = Vec::with_capacity(individual.speed.len());
    result.push(individual.points.first().unwrap().clone());
    result_speed.push(individual.speed.first().unwrap().clone());
    for (point, speed) in individual.points[1..(individual.points.len() - 1)].iter().zip(individual.speed[1..(individual.speed.len() - 1)].iter()) {
        if !ga.obstacles.static_obstacles.contains(point) {
            if range.sample(&mut rng) > 0.1 {
                continue;
            }
        }
        result.push(point.clone());
        result_speed.push(speed.clone());
    }
    result.push(individual.points.last().unwrap().clone());
    result_speed.push(individual.speed.last().unwrap().clone());
    assert!(result.len() == result_speed.len());
    individual.points = result;
    individual.speed = result_speed;
}

// TODO: Instead of doing this for every line segment choose only one.
pub fn insert_if_invalid_mutation(ga: &GeneticAlgorithm, individual: &mut Individual) {
    // println!("insert_if_invalid_mutation");

    let mut rng = thread_rng();
    let lines = LineString::new(individual.points.clone());
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    let mut result_speed: Vec<f64> = Vec::with_capacity(individual.speed.len());
    for (idx,line) in lines.lines().enumerate() {
        result.push(line.start_point().0);
        result_speed.push(individual.speed[idx]);
        if line.intersects(&ga.obstacles.static_obstacles) {
            // let index = individual
            //     .points
            //     .iter()
            //     .position(|point| *point == line.start_point().0)
            //     .unwrap();
            if ga.obstacles.static_obstacles.contains(&line.start)
                && ga.obstacles.static_obstacles.contains(&line.end)
            {
                continue;
            }

            let line_delta = line.delta();
            let middle_point: Coordinate = (
                line.start_point().x() + line_delta.x / 2.,
                line.start_point().y() + line_delta.y / 2.,
            )
                .into();

            let point_outside_polygons = {
                let mut x: [f64; 2] = UnitDisc.sample(&mut rng);
                x.iter_mut().for_each(|coord| {
                    if coord.abs() < 0.1 {
                        *coord = 0.1 * coord.signum();
                    }
                });

                let mut point: Coordinate = (
                    middle_point.x
                        + (x[0] * (line.euclidean_length()).max(ga.enviroment.width / 10.)),
                    middle_point.y
                        + (x[1] * (line.euclidean_length()).max(ga.enviroment.height / 10.)),
                )
                    .into();
                while ga.obstacles.static_obstacles.contains(&point) {
                    // println!("point: {point:.2?}, line: {line:.2?}, middle_point: {middle_point:.2?}, disc_sample: {x:.2?}, line_len: {:.2?}", line.euclidean_length());
                    x = UnitDisc.sample(&mut rng);
                    x.iter_mut().for_each(|coord| {
                        if coord.abs() < 0.1 {
                            *coord = 0.1 * coord.signum();
                        }
                    });
                    point = (
                        middle_point.x
                            + (x[0] * (line.euclidean_length()).max(ga.enviroment.width / 10.)),
                        middle_point.y
                            + (x[1] * (line.euclidean_length()).max(ga.enviroment.height / 10.)),
                    )
                        .into();
                }
                point
            };
            result.push(point_outside_polygons);
            result_speed.push(ga.config.individual_speed_values.choose(&mut rng).unwrap().clone());
        }
    }
    result.push(individual.points.last().unwrap().clone());
    result_speed.push(individual.speed.last().unwrap().clone());
    assert!(result.len() == result_speed.len());
    individual.points = result;
    individual.speed = result_speed;
}

pub fn shorten_path_mutate(ga: &GeneticAlgorithm, individual: &mut Individual) {
    // println!("shorten_path_mutate");
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    let mut result_speed: Vec<f64> = Vec::with_capacity(individual.speed.len());
    let points = &individual.points;
    result.push(points.first().unwrap().clone());
    result_speed.push(individual.speed.first().unwrap().clone());
    let line_string = LineString::new(individual.points.clone());
    let mut idx = 0usize;
    while idx < (line_string.0.len() - 2) {
        let mut temp = idx + 2;
        let line_start = line_string.0[idx];
        while temp < line_string.0.len() {
            let line = Line::new(line_start, line_string.0[temp]);
            // let multiline = MultiLineString::new(vec![line.into()]);
            // let intersection = multiline.intersection(&ga.obstacles.static_obstacles, 1000.);
            // if intersection.euclidean_length() > 0.{
            //     result.push(points[idx + 1]);
            //     break;
            // }
            let crossing_points = find_collision_point(&line_string![line_start, line_string.0[temp]], &ga.obstacles, &ga.enviroment);
            let temp_individual = Individual{
                fitness: 0.,
                feasible: false,
                dynamic_feasible: false,
                points: vec![line_start, line_string.0[temp]],
                evaluated: false,
                speed: vec![individual.speed[idx + 1], 1.],
            };
            let intersections = check_collision_with_dynamic_obstacles(&ga, &crossing_points, &temp_individual);

            if line.intersects(&ga.obstacles.static_obstacles) || intersections.euclidean_length() > 0. {
                result.push(points[idx + 1]);
                result_speed.push(individual.speed[idx + 1]);
                break;
            }
            idx += 1;
            temp += 1;
        }
        idx += 1;
    }
    result.push(points.last().unwrap().clone());
    result_speed.push(individual.speed.last().unwrap().clone());
    assert!(result.len() == result_speed.len());
    individual.points = result;
    individual.speed = result_speed;
}

// FIXME: Need to check if the line doesnt end inside an obstacle, if it does fetch more points until it finds one not inside anything and use that.
pub fn repair_mutation(ga: &GeneticAlgorithm, individual: &mut Individual) {
    // println!("repair_mutation");

    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    let mut result_speed: Vec<f64> = Vec::with_capacity(individual.speed.len());
    let line_string = LineString::new(individual.points.clone());
    let mut rng = thread_rng();

    result.push(individual.points.first().unwrap().clone());
    result_speed.push(individual.speed.first().unwrap().clone());
    let mut run_once = false;
    let mut line_iter = line_string.lines().enumerate();
    // for (i,line) in line_string.lines().enumerate() {
    while let Some((mut i, mut line)) = line_iter.next() {
        if line.intersects(&ga.obstacles.static_obstacles) {
            if run_once {
                result.push(line.end);
                result_speed.push(individual.speed[i]);
                continue;
            }
            run_once = true;
            let start_i = i;
            let mut visibility_graph = ga.obstacles.visibility_graph.clone();
            let start_node = visibility_graph.add_node(line.start);
            let starting_line_point = line.start;
            while ga.obstacles.static_obstacles.contains(&line.end)
                || starting_line_point == line.end
            {
                (i, line) = line_iter.next().unwrap_or_else(|| {
                    todo!();
                });
            }
            // let mut visibility_graph = ga.obstacles.visibility_graph.clone();
            // let start_node = visibility_graph.add_node(line.start);
            let end_node = visibility_graph.add_node(line.end);
            let (sender, receiver) = flume::unbounded();
            let new_edges: Vec<_> = {
                ga.obstacles
                    .static_obstacles_with_offset
                    .exterior_coords_iter()
                    .par_bridge()
                    .for_each_with(sender, |s, coord| {
                        let test_line1 = Line::new(starting_line_point, coord);
                        let test_line2 = Line::new(line.end, coord);
                        if !test_line1.intersects(&ga.obstacles.static_obstacles) {
                            s.send((starting_line_point, coord)).unwrap();
                        }
                        if !test_line2.intersects(&ga.obstacles.static_obstacles) {
                            s.send((line.end, coord)).unwrap();
                        }
                    });
                let edge_vec: Vec<_> = receiver.iter().collect();
                let (idx_sender, idx_receiver) = flume::unbounded();
                edge_vec.par_iter().for_each_with(idx_sender, |s, edge| {
                    let end_idx = visibility_graph
                        .node_indices()
                        .find(|i| visibility_graph[*i] == edge.1)
                        .unwrap();
                    if edge.0 == starting_line_point {
                        s.send((
                            start_node,
                            end_idx,
                            Line::new(edge.0, edge.1).euclidean_length(),
                        ))
                        .unwrap();
                    } else {
                        s.send((
                            end_node,
                            end_idx,
                            Line::new(edge.0, edge.1).euclidean_length(),
                        ))
                        .unwrap();
                    }
                });
                idx_receiver.iter().collect()
            };
            visibility_graph.extend_with_edges(new_edges);
            let detour_res = astar(
                &visibility_graph,
                start_node,
                |e| e == end_node,
                |e| *e.weight(),
                |_| 0.,
            );
            let detour = match detour_res {
                Some(x) => x,
                None => {
                    eprintln!(
                        "A* detour not found: start_i {}, line {}, start: {:?}, end: {:?}",
                        start_i, i, starting_line_point, line.end
                    );
                    draw_env_to_file(
                        "debug_repair.png",
                        &ga.obstacles,
                        &ga.enviroment,
                        &individual.points,
                        None,
                        None,
                        None,
                    )
                    .unwrap();
                    todo!()
                }
            };
            let mut detour_coords = Vec::with_capacity(detour.1.len());
            // println!("points {detour:#?}");
            for path_node in detour.1.iter().skip(1) {
                detour_coords.push(visibility_graph.node_weight(*path_node).unwrap().clone());
                result_speed.push(ga.config.individual_speed_values.choose(&mut rng).unwrap().clone());
            }
            
            result.extend(detour_coords);
            continue;
        }
        result.push(line.end);
        result_speed.push(individual.speed[i]);
    }
    assert!(result.len() == result_speed.len());
    individual.points = result;
    individual.speed = result_speed;
}

pub fn speed_mutate(ga: &GeneticAlgorithm, individual: &mut Individual){
    let path = LineString::new(individual.points.clone());
    for (idx,line) in path.lines().enumerate(){
        let crossing_points = find_collision_point(&line_string![line.start, line.end], &ga.obstacles, &ga.enviroment);
        let temp_individual = Individual{
            fitness: 0.,
            feasible: false,
            dynamic_feasible: false,
            points: vec![line.start, line.end],
            evaluated: false,
            speed: vec![individual.speed[idx], 1.],
        };
        let intersections = check_collision_with_dynamic_obstacles(&ga, &crossing_points, &temp_individual);
        if intersections.euclidean_length() > 0. || random::<bool>(){
            let increase_speed: bool = random();
            let speed_idx = ga.config.individual_speed_values.iter().position(|a| *a == individual.speed[idx]).unwrap();
            if increase_speed{
                if speed_idx == ga.config.individual_speed_values.len() - 1{
                    individual.speed[idx] = ga.config.individual_speed_values[speed_idx - 1];
                }else{
                    individual.speed[idx] = ga.config.individual_speed_values[speed_idx + 1];
                }
            }else{
                if speed_idx == 0{
                    individual.speed[idx] = ga.config.individual_speed_values[speed_idx + 1];
                }else{
                    individual.speed[idx] = ga.config.individual_speed_values[speed_idx - 1];
                }
            }
        }
    }
}