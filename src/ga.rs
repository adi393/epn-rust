use core::panic;
use anyhow::anyhow;
use std::{
    f64::consts::PI,
    ops::Not,
    sync::{
        mpsc::{channel, Receiver, Sender},
        Arc,
    },
};

use anyhow::Result;
use geo::{
    Closest, ClosestPoint, Contains, Coordinate, EuclideanLength, Intersects, Line, LineString,
    MultiLineString, MultiPolygon, Polygon, CoordsIter,
};
use geo_clipper::ClipperOpen;
use petgraph::{graph::{UnGraph, self}, visit::IntoNodeReferences};
use petgraph::algo::astar;
use rand::{
    distributions::Uniform,
    prelude::{Distribution, SliceRandom},
    thread_rng, seq::index,
};
use rand_distr::{UnitCircle, UnitDisc};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator, ParallelBridge};
use wkt::ToWkt;

use crate::debug_draw_visibility_graph;
#[derive(Debug, Clone, PartialEq)]
pub struct Individual {
    pub fitness: f64,
    pub feasible: bool,
    pub points: Vec<Coordinate<f64>>,
}

impl Individual {
    pub fn new(points: Vec<Coordinate<f64>>) -> Individual {
        Individual {
            fitness: 1e10,
            feasible: false,
            points,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Obstacles {
    pub static_obstacles: MultiPolygon<f64>,
    pub static_obstacles_with_offset: MultiPolygon<f64>,
    // pub dynamic_obstacles: Vec::<DynamicObstacle>,
    pub visibility_graph: UnGraph<Coordinate, f64, usize>,
}
#[derive(Debug, Clone)]
pub struct DynamicObstacle {
    pub polygon: Polygon<f64>,
    pub direction_angle: f64,
    pub speed: f64,
}
#[derive(Debug, Clone)]
pub struct Enviroment {
    pub width: f64,
    pub height: f64,
}

pub fn evaluate(individual: &Individual, obstacles: &Obstacles) -> f64 {
    let path = MultiLineString::new(vec![individual.points.clone().into()]);
    let path_length = path.euclidean_length();
    let test = &obstacles.static_obstacles.0;
    let (sender, receiver): (Sender<f64>, Receiver<f64>) = channel();
    test.par_iter().for_each_with(sender, |sender, poly| {
        if poly.intersects(&path) {
            let intersection = path.intersection(poly, 1.);
            sender.send(intersection.euclidean_length()).unwrap();
        }
    });
    let mut angle_cost = 0.;

    for slice in individual.points.windows(3) {
        if let [p1, p2, p3] = slice {
            let angle1 = (p2.y - p1.y).atan2(p2.x - p1.x);
            let angle2 = (p2.y - p3.y).atan2(p2.x - p3.x);
            let result = {
                let mut x = (angle2 - angle1) * 180. / PI;
                if x < 0. {
                    x += 360.;
                }
                if x > 180. {
                    x = 360. - x;
                }
                x
            };
            // println!("Line angle: {result}");
            angle_cost += (180. - result).powf(2.);
        }
    }

    let length_in_objects: f64 = receiver.iter().sum();
    path_length + length_in_objects * 100. + angle_cost
}

pub fn hard_mutation(individual: &mut Individual, enviroment: &Enviroment) {
    let mut rng = thread_rng();
    let len = individual.points.len();
    let env_width_range = Uniform::new(0., enviroment.width);
    let env_height_range = Uniform::new(0., enviroment.height);
    let mut element = individual.points[1..len - 1].choose_mut(&mut rng).unwrap();
    element.x = env_width_range.sample(&mut rng);
    element.y = env_height_range.sample(&mut rng);
}

pub fn swap_mutation(individual: &mut Individual) {
    let mut rng = thread_rng();
    let range = Uniform::from(1..individual.points.len() - 1);
    let (mut p1, mut p2) = (0, 0);
    while p1 == p2 {
        p1 = range.sample(&mut rng);
        p2 = range.sample(&mut rng);
    }
    individual.points.swap(p1, p2);
}

pub fn move_mutation(individual: &mut Individual, obstacles: &Obstacles) {
    let mut rng = thread_rng();
    let range = Uniform::new(0., 1.);
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    for point in individual.points[1..individual.points.len()].iter() {
        for poly in obstacles.static_obstacles.iter() {
            if poly.contains(point) {
                // let min_dist = poly.exterior().euclidean_distance(&Point::new(point.x, point.y));
                let closest = match poly.exterior().closest_point(&point.clone().into()) {
                    Closest::Intersection(val) => val,
                    Closest::SinglePoint(val) => val,
                    Closest::Indeterminate => panic!("die i guess?"),
                };

                let random_point_on_circle: [f64; 2] = UnitCircle.sample(&mut rng);
                result.push(
                    (
                        closest.x() + random_point_on_circle[0],
                        closest.y() + random_point_on_circle[1],
                    )
                        .into(),
                );
            }
        }
        result.push(point.clone());
    }
    individual.points = result;
}

// This mutation loops over all points and rolls a 20% chance to remove it if it is not inside any polygons.
pub fn delete_mutation(individual: &mut Individual, obstacles: &Obstacles) {
    let mut rng = thread_rng();
    let range = Uniform::new(0., 1.);
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    for point in individual.points[1..individual.points.len()].iter() {
        if !obstacles.static_obstacles.contains(point) {
            if range.sample(&mut rng) > 0.2 {
                continue;
            }
        }
        result.push(point.clone());
    }
    individual.points = result;
}

// TODO: Instead of doing this for every line segment choose only one.
pub fn insert_if_invalid_mutation(individual: &mut Individual, obstacles: &Obstacles) {
    let mut rng = thread_rng();
    let lines = LineString::new(individual.points.clone());
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    for line in lines.lines() {
        result.push(line.start_point().0);
        if line.intersects(&obstacles.static_obstacles) {
            // let index = individual
            //     .points
            //     .iter()
            //     .position(|point| *point == line.start_point().0)
            //     .unwrap();

            let random_point_in_disc: [f64; 2] = UnitDisc.sample(&mut rng);
            let line_delta = line.delta();
            let middle_point: Coordinate = (
                line.start_point().x() + line_delta.x / 2.,
                line.start_point().y() + line_delta.y / 2.,
            )
                .into();
            result.push(
                (
                    middle_point.x + random_point_in_disc.first().unwrap(),
                    middle_point.y + random_point_in_disc.last().unwrap(),
                )
                    .into(),
            );
        }
    }
    result.push(individual.points.last().unwrap().clone());
}

pub fn two_point_crossover(individual: [&mut Individual; 2]) {
    let mut rng = thread_rng();
    let lengths: Vec<_> = individual.iter().map(|ind| ind.points.len()).collect();
    let range = {
        let min_len = lengths.iter().min().unwrap();
        Uniform::new(1, min_len - 1)
    };

    let c: Vec<_> = range.sample_iter(&mut rng).take(2).collect();
    let crossover_low = c.iter().min().unwrap();
    let crossover_high = c.iter().max().unwrap();

    let mut result1 = Vec::<Coordinate>::with_capacity(*crossover_high);
    let mut result2 = Vec::<Coordinate>::with_capacity(*crossover_high);

    // Copy the part before crossover
    result1.extend_from_slice(&individual[0].points[0..*crossover_low]);
    result2.extend_from_slice(&individual[1].points[0..*crossover_low]);

    // Copy the crossover part from opposite individuals
    result1.extend_from_slice(&individual[1].points[*crossover_low..*crossover_high]);
    result2.extend_from_slice(&individual[0].points[*crossover_low..*crossover_high]);

    // Copy the remaining points.
    result1.extend_from_slice(&individual[0].points[*crossover_high..]);
    result2.extend_from_slice(&individual[1].points[*crossover_high..]);

    individual[0].points = result1;
    individual[1].points = result2;
}

pub fn shorten_path_mutate(individual: &mut Individual, obstacles: &Obstacles) {
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    let points = &individual.points;
    result.push(points.first().unwrap().clone());
    let line_string = LineString::new(individual.points.clone());
    let mut idx = 0usize;
    while idx < (line_string.0.len() - 2) {
        let mut temp = idx + 2;
        let line_start = line_string.0[idx];
        while temp < line_string.0.len() {
            let line = Line::new(line_start, line_string.0[temp]);
            if line.intersects(&obstacles.static_obstacles) {
                result.push(points[idx + 1]);
                break;
            }
            idx += 1;
            temp += 1;
        }
        idx += 1;
    }
    result.push(points.last().unwrap().clone());
    individual.points = result;
}

pub fn repair_mutation(individual: &mut Individual, obstacles: &Obstacles) {
    let mut result: Vec<Coordinate> = Vec::with_capacity(individual.points.len());
    let line_string = LineString::new(individual.points.clone());
    
    result.push(individual.points.first().unwrap().clone());
    let mut run_once = false;

    for line in line_string.lines() {
        if line.intersects(&obstacles.static_obstacles) {
            if run_once{
                result.push(line.end);
                continue;
            }
            run_once = true;
            let mut visibility_graph = obstacles.visibility_graph.clone();
            let start_node = visibility_graph.add_node(line.start);
            let end_node = visibility_graph.add_node(line.end);
            let (sender, receiver) = channel();
            let new_edges: Vec<_> = {
                obstacles.static_obstacles_with_offset.exterior_coords_iter().par_bridge().for_each_with(sender,|s, coord|{
                    
                    let test_line1 = Line::new(line.start, coord);
                    let test_line2 = Line::new(line.end, coord);
                    if !test_line1.intersects(&obstacles.static_obstacles){ s.send((line.start, coord)).unwrap();}
                    if !test_line2.intersects(&obstacles.static_obstacles){ s.send((line.end, coord)).unwrap();}
                });
                let edge_vec: Vec<_> = receiver.iter().collect();
                let (idx_sender, idx_receiver) = channel();
                edge_vec.par_iter().for_each_with(idx_sender, |s, edge|{
                    let end_idx = visibility_graph.node_indices().find(|i| visibility_graph[*i] == edge.1 ).unwrap();
                    if edge.0 == line.start{
                        s.send((start_node, end_idx, Line::new(edge.0, edge.1).euclidean_length())).unwrap();
                    } else{
                        s.send((end_node, end_idx, Line::new(edge.0, edge.1).euclidean_length())).unwrap();
                    }
                });
                idx_receiver.iter().collect()
            };
            visibility_graph.extend_with_edges(new_edges);
            let detour = astar(&visibility_graph, start_node, |e| e == end_node, |e| *e.weight(), |_| 0.).unwrap();
            let mut detour_coords = Vec::with_capacity(detour.1.len());
            // println!("points {detour:#?}");
            for path_node in detour.1.iter().skip(1){
                detour_coords.push(visibility_graph.node_weight(*path_node).unwrap().clone());
            }
            result.extend(detour_coords);
            continue;
        }
        result.push(line.end);
    }
    individual.points = result;

}

pub fn roulette_selector(population: &Vec<Individual>, selection_count: usize) -> Result<Vec<Individual>>{
    let mut rng = thread_rng();
    let fitness_sum = population.iter().fold(0., |acc, ind|{ acc + ind.fitness });
    let mut probability_sum = 0.;
    let mut probability_vec: Vec<f64> = Vec::with_capacity(population.len());
    let mut result = Vec::with_capacity(selection_count);
    for individual in population.iter(){
        probability_vec.push(individual.fitness / fitness_sum);
    }
    // population.iter().for_each(|x| println!("fitness: {}", x.fitness));
    let mut selection_wheel_probability: Vec<f64> = Vec::default();
    for probability in probability_vec.iter(){
        probability_sum += (1. - probability) / (population.len() - 1) as f64;
        selection_wheel_probability.push(probability_sum);
    }
    // selection_wheel_probability.iter().for_each(|x| println!("selection prob: {}", x));

    let between = Uniform::new(0., 1.);
    for _ in 0..selection_count{
        let roll = between.sample(&mut rng);
        for (idx, probability) in selection_wheel_probability.iter().enumerate(){
            if roll < *probability{
                result.push(population[idx].clone());
                break;
            }
        }
    }

    Ok(result)
}

// Selects `selection_count` individuals from old population in `selection_count / 2` tournaments consisting
// of `participants_count` participants
pub fn tournament_selector(population: &Vec<Individual>, selection_count: usize, participants_count: usize) -> Result<Vec<Individual>>{
    if selection_count == 0 || selection_count % 2 != 0 || selection_count >= population.len(){
        return Err(anyhow!("Invalid parameter `selection_count`: {}. Should be higher than 0, less than population size and a multiple of two.", selection_count));
    }
    if participants_count < 2 || participants_count >= population.len(){
        return Err(anyhow!("Invalid parameter `participants_count`: {}. Should be higher than 2 and less than population size.", participants_count));
    }
    let mut result: Vec<Individual> = Vec::with_capacity(selection_count);
    let mut rng = thread_rng();

    for _ in 0..(selection_count / 2){
        let mut tournament: Vec<Individual> = population.choose_multiple(&mut rng, participants_count).cloned().collect();
        tournament.sort_by(|a,b| a.fitness.total_cmp(&b.fitness));
        result.extend_from_slice(&tournament[0..2]);
    } 
    Ok(result)
}

pub fn uniform_selector(population: &Vec<Individual>, selection_count: usize) -> Result<Vec<Individual>>{
    let mut rng = thread_rng();
    let mut result = Vec::with_capacity(selection_count);
    let indexes = rand::seq::index::sample(&mut rng, population.len(), selection_count);
    for idx in indexes{
        result.push(population[idx].clone());
    }
    Ok(result)
}

pub fn stochastic_universal_sampling_selector(population: &Vec<Individual>, selection_count: usize) -> Result<Vec<Individual>>{
    let mut minimize_fitness_vec: Vec<f64> = Vec::with_capacity(population.len());
    let max_fitness: f64 = population.iter().max_by(|x,y| x.fitness.total_cmp(&y.fitness)).unwrap().fitness;
    let bias = max_fitness / 6.;
    for ind in population.iter(){
        minimize_fitness_vec.push(max_fitness - ind.fitness + bias);
        // println!("minimize_fitness: {}", max_fitness - ind.fitness + bias);
    }
    let minimize_fitness_sum: f64 = minimize_fitness_vec.iter().sum();
    let mut current_point = minimize_fitness_vec.first().unwrap().clone();
    let chunk = minimize_fitness_sum / selection_count as f64;
    let mut idx = 0;
    let mut result = Vec::with_capacity(selection_count);
    // println!("chunk: {chunk:.4}, minimize_fitness_sum: {minimize_fitness_sum}");
    for i in 0..selection_count{
        'inner: loop{
            if (i as f64 + 1.) * chunk <= (current_point + 0.01){
                result.push(population[idx].clone());
                break 'inner;
            }
            else{
                idx += 1;
                current_point += minimize_fitness_vec[idx];
            }
        }
    }
    Ok(result)
}
